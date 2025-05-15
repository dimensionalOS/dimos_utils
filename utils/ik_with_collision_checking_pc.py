#!/usr/bin/env python3

import numpy as np
import time
import os
import lcm
import argparse
import sys
import tqdm
import trimesh.primitives
import open3d as o3d
from importlib.machinery import SourceFileLoader
import xml.etree.ElementTree as ET

# Fix LCM message imports
try:
    from lcm_msgs.sensor_msgs import JointState
except ImportError:
    try:
        # Try to load the message from the lcm_dimos_msgs project
        lcm_msgs = SourceFileLoader('lcm_msgs', '/home/yashas/Documents/dimensional/lcm_dimos_msgs/python_lcm_msgs/lcm_msgs/__init__.py').load_module()
        from lcm_msgs.sensor_msgs import JointState
    except ImportError:
        print("WARNING: Could not import JointState from lcm_msgs.sensor_msgs")

from pydrake.all import (
    MultibodyPlant, Parser, InverseDynamics, SceneGraph,
    DiagramBuilder, MathematicalProgram, Solve, RigidTransform,
    PiecewisePolynomial, TrajectorySource, 
    Simulator, RotationMatrix, RollPitchYaw, SpatialVelocity,
    AutoDiffXd, AddMultibodyPlantSceneGraph, JacobianWrtVariable,
    FindResourceOrThrow, eq, ge, le, GeometryInstance, Sphere,
    MakePhongIllustrationProperties, Rgba, CollisionFilterDeclaration,
    MeshcatVisualizer, StartMeshcat
)
import matplotlib.pyplot as plt
from tqdm import tqdm

class URDFInverseKinematicsWithCollisionChecker:
    def __init__(self, urdf_path, end_effector_link, kinematic_chain_joints, point_cloud=None, point_cloud_radius=0.01, visualize=True):
        """
        Initialize the IK solver with collision checking.
        
        Args:
            urdf_path: Path to the URDF file
            end_effector_link: Name of end effector link
            kinematic_chain_joints: List of joint names to include in the kinematic chain
        """
        self.urdf_path = urdf_path
        self.end_effector_link = end_effector_link
        self.kinematic_chain_joints = kinematic_chain_joints
        # Store point cloud info for later use
        self.point_cloud_geometries = []
        self.point_cloud = point_cloud
        self.point_cloud_radius = point_cloud_radius or 0.01  # Default radius if not specified
        self.point_cloud_trimesh_spheres = []
        self.visualize = visualize
        self.meshcat = None
        
        # Initialize meshcat if visualization is enabled
        if self.visualize:
            self.meshcat = StartMeshcat()
        
        # Initialize plant with collision detection
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.0)
        
        # Load URDF model
        self.parser = Parser(self.plant)
        model_instances = self.parser.AddModelsFromUrl(f"file://{self.urdf_path}")
        # We'll use the first model instance returned
        self.model_instance = model_instances[0] if model_instances else None
        
        # Finalize the plant
        self.plant.Finalize()
        
        # Get joint indices for the kinematic chain
        self.joint_indices = []
        for joint_name in self.kinematic_chain_joints:
            try:
                joint = self.plant.GetJointByName(joint_name)
                if joint.num_positions() > 0:  # Only add if it's an actuated joint
                    self.joint_indices.extend(
                        range(joint.position_start(), joint.position_start() + joint.num_positions())
                    )
            except RuntimeError:
                print(f"Warning: Joint {joint_name} not found in the URDF")
        
        # Get end effector body
        try:
            self.end_effector_body = self.plant.GetBodyByName(self.end_effector_link)
        except RuntimeError:
            print(f"Error: End effector link {self.end_effector_link} not found in the URDF")
            raise
        
        # Add visualization if enabled
        if self.visualize:
            # Add MeshcatVisualizer to the diagram
            self.visualizer = MeshcatVisualizer.AddToBuilder(
                builder=self.builder,
                scene_graph=self.scene_graph,
                meshcat=self.meshcat
            )
        
        # Set some basic visualization settings
        if self.meshcat:
            # Clear existing objects but do not reset the camera
            try:
                # In newer versions of Drake, we can use DeletePrefix to preserve camera settings
                self.meshcat.DeletePrefix("/drake")
            except AttributeError:
                # Fall back to Delete, which may reset camera
                self.meshcat.Delete()
                print("Note: Using older Meshcat API, camera view may be reset")
                
            # We intentionally avoid setting the camera to allow interactive control
            # Simply provide a note for the user about camera controls
            print("Note: You can interact with the 3D view using mouse controls:")
            print("  - Left click + drag: Rotate the camera")
            print("  - Right click + drag: Pan the camera")
            print("  - Scroll wheel: Zoom in/out")
        
        # Build the system and create a context
        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)
        
        # With the context created, we're ready for collision detection
        # The point cloud will be used directly for collision detection
        # rather than adding to the scene graph (which is already finalized)
        if self.point_cloud is not None and len(self.point_cloud) > 0:
            print(f"Using {len(self.point_cloud)} points for collision detection with radius {self.point_cloud_radius}m")
        
        # Set up visualization default view
        if self.visualize and self.meshcat:
            print("\nMeshcat visualization available at http://localhost:7000")
            print("Please open this URL in your browser to view the 3D visualization.")
            # For now, we'll skip explicit camera setup as API varies between Drake versions
            # The default camera will be used
        
        # Store joint limits
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        for idx in self.joint_indices:
            self.joint_lower_limits.append(self.plant.GetPositionLowerLimits()[idx])
            self.joint_upper_limits.append(self.plant.GetPositionUpperLimits()[idx])
            
        # Store collision pairs to ignore (will be populated by calibration)
        self.collision_pairs_to_ignore = set()
    
    def set_joint_positions(self, q):
        """Set joint positions in the plant context."""
        full_q = self.plant.GetPositions(self.plant_context)
        for i, idx in enumerate(self.joint_indices):
            full_q[idx] = q[i]
        self.plant.SetPositions(self.plant_context, full_q)
    
    def get_random_joint_positions(self):
        """Generate random joint positions within limits."""
        q = np.zeros(len(self.joint_indices))
        for i in range(len(self.joint_indices)):
            q[i] = np.random.uniform(self.joint_lower_limits[i], self.joint_upper_limits[i])
        return q
    
    def add_point_cloud_to_scene(self, points, radius):
        """Add a point cloud to the scene for visualization."""
        # Get the scene graph from the diagram
        source_id = self.plant.get_source_id()
        
        for i, point in enumerate(points):
            # Create a sphere at the point location
            sphere = Sphere(radius)
            instance = GeometryInstance(
                RigidTransform(point),  # Position the sphere at the point
                sphere,                 # The sphere
                f"point_cloud_sphere_{i}"  # Unique name for the geometry
            )
            
            # Add visual properties with direct rgba values as numpy array
            orange = np.array([1.0, 0.55, 0.0, 0.5])  # RGBA format
            instance.set_illustration_properties(
                MakePhongIllustrationProperties(orange)
            )
            
            # Register the geometry with the scene (for visualization only)
            geometry_id = self.plant.RegisterCollisionGeometry(
                self.plant.world_body(),
                RigidTransform(point),
                sphere,
                f"point_cloud_sphere_{i}",
                CoulombFriction(0.9, 0.8))
            self.point_cloud_geometries.append(geometry_id)
            
            # If we have too many points, just warn and stop
            if i >= 1000:
                print(f"Warning: Limited point cloud visualization to 1000 points for performance")
                break
        
        print(f"Added {min(len(points), 1000)} points to the scene")
    
    def create_point_cloud_collision_spheres(self):
        """Create trimesh spheres for each point in the point cloud for collision detection"""
        print(f"Creating collision spheres for {len(self.point_cloud)} points")
        
        # Create a trimesh sphere for each point with the specified radius
        for i, point in enumerate(self.point_cloud):
            sphere = trimesh.primitives.Sphere(radius=self.point_cloud_radius, center=point)
            self.point_cloud_trimesh_spheres.append(sphere)
            
            # Limit the number of spheres for performance
            if i >= 1000:
                print(f"Warning: Limited point cloud collision spheres to 1000 for performance")
                break
        
        print(f"Created {len(self.point_cloud_trimesh_spheres)} collision spheres")
    
    def check_self_collisions(self):
        """Check for self-collisions in the current configuration."""
        try:
            query_port = self.scene_graph.GetOutputPort("query")
            query_object = query_port.Eval(self.scene_graph.GetMyContextFromRoot(self.diagram_context))
            
            # Get collision pairs
            collision_pairs = query_object.ComputePointPairPenetration()
        except Exception as e:
            print(f"Error in collision detection: {e}")
            return []
        
        # Filter out ignored collision pairs
        filtered_pairs = []
        for pair in collision_pairs:
            # In this version of Drake, we'll use the geometry IDs directly as a unique identifier
            # This is a simplification but should work for our collision checking needs
            body_A_id = pair.id_A
            body_B_id = pair.id_B
            
            collision_key = (str(body_A_id), str(body_B_id))
            reverse_key = (str(body_B_id), str(body_A_id))
            
            # Skip if in ignored pairs
            if collision_key in self.collision_pairs_to_ignore or reverse_key in self.collision_pairs_to_ignore:
                continue
                
            filtered_pairs.append(pair)
        
        return filtered_pairs
        
    def check_point_cloud_collisions(self):
        """Check for collisions between robot links and point cloud"""
        if self.point_cloud is None or len(self.point_cloud) == 0:
            return []
        
        # Get all robot link positions and bounding spheres
        robot_links = []
        
        # Iterate through bodies using indices properly
        for body_index in self.plant.GetBodyIndices(self.model_instance):
            body = self.plant.get_body(body_index)
            
            # Skip the world body and fixed base for efficiency
            if body.name() == "world" or body.name() == "base_center":
                continue
            
            # Get the pose of this body in world frame
            body_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, body)
            position = body_pose.translation()
            
            # Add to our list with a conservative bounding radius
            # We use a large enough radius based on the robot's link sizes
            robot_links.append({
                'name': body.name(),
                'position': position,
                'radius': 0.15  # Conservative radius for all links
            })
        
        # Direct collision check between robot links and point cloud points
        collisions = []
        
        # For each point in the point cloud
        for i, point in enumerate(self.point_cloud):
            # Skip if we already have many collisions (for performance)
            if len(collisions) > 10:
                print(f"Found {len(collisions)} collisions, skipping remaining point cloud points")
                break
                
            # Check against each robot link
            for link in robot_links:
                # Calculate distance between point and link center
                distance = np.linalg.norm(point - link['position'])
                
                # Check if distance is less than sum of radii (collision)
                if distance < (link['radius'] + self.point_cloud_radius):
                    # Calculate penetration depth
                    depth = (link['radius'] + self.point_cloud_radius) - distance
                    
                    # Record collision
                    collision = {
                        'body_name': link['name'],
                        'point_index': i,
                        'depth': depth,
                        'point_pos': point,
                        'link_pos': link['position']
                    }
                    collisions.append(collision)
                    # Skip other links for this point once we found a collision
                    break
        
        return collisions
        

    def visualize_collision_points(self, collision_pairs):
        """Visualize collision points in red.
        
        Args:
            collision_pairs: List of collision pairs from check_self_collisions or from get_collision_geometry_names
        """
        if not self.visualize or not self.meshcat:
            return
            
        # Clear existing collision markers
        try:
            self.meshcat.Delete("/collision_markers")
        except:
            pass
            
        # Create a bright red color for collision points
        red = np.array([1.0, 0.0, 0.0, 1.0])  # RGBA format - bright red
        
        # Counter for marker naming
        marker_count = 0
        
        # Process dictionary-format collision pairs (from get_collision_geometry_names)
        if collision_pairs and isinstance(collision_pairs[0], dict):
            for collision in collision_pairs:
                # Extract position information from frame_A and frame_B
                for frame_info in [collision['frame_A'], collision['frame_B']]:
                    if 'Position' in frame_info:
                        # Extract the position vector using a simple regex-like approach
                        pos_str = frame_info.split('Position ')[1]
                        if pos_str.startswith('[') and ']' in pos_str:
                            pos_str = pos_str[1:pos_str.find(']')]
                            try:
                                # Convert string representation to numpy array
                                pos_values = [float(x) for x in pos_str.split()]
                                if len(pos_values) == 3:
                                    position = np.array(pos_values)
                                    
                                    # Create a small red sphere at the collision point
                                    marker_name = f"/collision_markers/point_{marker_count}"
                                    self.meshcat.SetObject(marker_name, 
                                                   Sphere(0.01),  # Small sphere
                                                   Rgba(red[0], red[1], red[2], red[3]))
                                    # Position the marker at the collision point
                                    self.meshcat.SetTransform(marker_name, RigidTransform(position))
                                    marker_count += 1
                            except:
                                pass  # Skip if we can't parse the position
        
        # Process raw collision pairs (from check_self_collisions)
        else:
            for pair in collision_pairs:
                # Get the collision points
                points = []
                if hasattr(pair, 'p_WCa') and pair.p_WCa is not None:
                    points.append(pair.p_WCa)
                if hasattr(pair, 'p_WCb') and pair.p_WCb is not None:
                    points.append(pair.p_WCb)
                    
                # Add a marker for each point
                for point in points:
                    marker_name = f"/collision_markers/point_{marker_count}"
                    self.meshcat.SetObject(marker_name, 
                                   Sphere(0.01),  # Small sphere
                                   Rgba(red[0], red[1], red[2], red[3]))
                    # Position the marker at the collision point
                    self.meshcat.SetTransform(marker_name, RigidTransform(point))
                    marker_count += 1
        
        print(f"Visualized {marker_count} collision points in red")

    def get_collision_geometry_names(self):
        """Get the names of geometries that are in collision."""
        collision_pairs = self.check_self_collisions()
        geometry_pairs = []
        
        # Load and parse the URDF to extract link names
        import xml.etree.ElementTree as ET
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            link_names = [link.attrib['name'] for link in root.findall('.//link')]
            print(f"\nAvailable links in the URDF: {', '.join(link_names[:5])}... (and {len(link_names)-5} more)")
        except Exception as e:
            print(f"Could not parse URDF file: {e}")
            link_names = []
        
        # Map GeometryId values to indices or offsets
        # This is just a guess, but often geometry IDs have some relationship to link indices
        geometry_id_mapping = {}
        for i, link_name in enumerate(link_names):
            # Create a few possible mappings for geometry IDs
            # Note: These are heuristics and may not be accurate
            base_id = (i+1) * 7  # Just a heuristic
            for offset in range(-3, 4):  # Try a few offsets around our guessed base ID
                potential_id = base_id + offset
                geometry_id_mapping[f"<GeometryId value={potential_id}>"] = link_name
        
        # Process each collision pair
        for pair in collision_pairs:
            # Get information from the collision pair
            geometry_A_id = str(pair.id_A)
            geometry_B_id = str(pair.id_B)
            penetration_depth = pair.depth
            
            # Extract numeric ID from the geometry ID string
            try:
                id_A_num = int(geometry_A_id.split('value=')[1].split('>')[0])
                id_B_num = int(geometry_B_id.split('value=')[1].split('>')[0])
            except:
                id_A_num = 0
                id_B_num = 0
            
            # Try to map to link names
            link_A = geometry_id_mapping.get(geometry_A_id, "")
            link_B = geometry_id_mapping.get(geometry_B_id, "")
            
            # If we don't have a direct mapping, try to guess based on the ID value
            if not link_A and len(link_names) > 0:
                # Try to associate with a link based on ID value
                try:
                    closest_idx = min(range(len(link_names)), key=lambda i: abs((i+1)*7 - id_A_num))
                    link_A = f"Possibly {link_names[closest_idx]}"
                except:
                    link_A = "Unknown link"
            
            if not link_B and len(link_names) > 0:
                try:
                    closest_idx = min(range(len(link_names)), key=lambda i: abs((i+1)*7 - id_B_num))
                    link_B = f"Possibly {link_names[closest_idx]}"
                except:
                    link_B = "Unknown link"
            
            # Get positions
            frame_A_pos = getattr(pair, 'p_WCa', None)
            frame_B_pos = getattr(pair, 'p_WCb', None)
            
            frame_A_info = f"Position {frame_A_pos}" if frame_A_pos is not None else "unknown position"
            frame_B_info = f"Position {frame_B_pos}" if frame_B_pos is not None else "unknown position"
            
            # Store collision information
            geometry_pairs.append({
                "geometry_A": geometry_A_id,
                "link_A": link_A,
                "frame_A": frame_A_info,
                "geometry_B": geometry_B_id,
                "link_B": link_B,
                "frame_B": frame_B_info,
                "depth": penetration_depth
            })

        self.visualize_collision_points(geometry_pairs)
        
        return geometry_pairs
    
    def get_joint_name_positions(self, q):
        """Get a dictionary mapping joint names to their current positions."""
        self.set_joint_positions(q)
        joint_positions = {}
        
        for i, joint_name in enumerate(self.kinematic_chain_joints):
            # This is a simplification - in reality, a joint might control multiple positions
            # We'll assume a 1:1 mapping here
            if i < len(q):
                joint_positions[joint_name] = q[i]
        
        return joint_positions
    
    def run_calibration(self, num_samples=10000, collision_threshold=0.95):
        """
        Run calibration to identify frequent self-collisions to ignore.
        
        Args:
            num_samples: Number of random configurations to sample
            collision_threshold: Threshold for ignoring collision pairs (0.0-1.0)
        """
        print(f"Running calibration with {num_samples} random samples...")
        
        # Dictionary to count collisions for each pair
        collision_counts = {}
        
        # Sample random configurations and check for collisions
        for _ in tqdm(range(num_samples)):
            q = self.get_random_joint_positions()
            self.set_joint_positions(q)
            
            # Evaluate collision pairs
            query_port = self.scene_graph.GetOutputPort("query")
            query_object = query_port.Eval(self.scene_graph.GetMyContextFromRoot(self.diagram_context))
            collision_pairs = query_object.ComputePointPairPenetration()
            
            # Count collision occurrences
            for pair in collision_pairs:
                body_A_id = pair.id_A
                body_B_id = pair.id_B
                
                # Create a unique identifier for this collision pair
                # Ensure consistent ordering of the pair
                id_A_str = str(body_A_id)
                id_B_str = str(body_B_id)
                if id_A_str > id_B_str:
                    id_A_str, id_B_str = id_B_str, id_A_str
                
                collision_key = (id_A_str, id_B_str)
                collision_counts[collision_key] = collision_counts.get(collision_key, 0) + 1
        
        # Identify collision pairs to ignore based on threshold
        for pair, count in collision_counts.items():
            if count / num_samples >= collision_threshold:
                self.collision_pairs_to_ignore.add(pair)
                print(f"Ignoring collision pair: {pair} (collision rate: {count/num_samples:.2f})")
        
        print(f"Calibration complete. Ignoring {len(self.collision_pairs_to_ignore)} collision pairs.")
        return self.collision_pairs_to_ignore
    
    def forward_kinematics(self, q, visualize=False):
        """
        Compute forward kinematics for the given joint positions.
        
        Args:
            q: Joint positions for the kinematic chain
            visualize: Whether to visualize this configuration
            
        Returns:
            end_effector_pose: RigidTransform representing the end effector pose
        """
        # Set the joint positions
        self.set_joint_positions(q)
        
        # Get the end effector transform
        end_effector_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
        
        if visualize and self.visualize and self.meshcat:
            # Update the visualization - force a fresh draw to ensure visibility
            self.diagram.ForcedPublish(self.diagram_context)
            
            # Add a small delay to allow visualization to render
            time.sleep(0.1)
            
        return end_effector_pose
    
    def inverse_kinematics(self, target_pose, initial_guess=None, visualize_steps=False):
        """
        Solve inverse kinematics to reach the target pose using a numerical approach.
        
        Args:
            target_pose: Target RigidTransform for the end effector
            initial_guess: Initial joint positions (if None, use random values)
            visualize_steps: If True, visualize intermediate steps
            
        Returns:
            q_sol: Solution joint positions
            info: Information about the solution (success, etc)
        """
        # Set initial guess for optimization
        if initial_guess is None:
            q = self.get_random_joint_positions()
        else:
            q = initial_guess.copy()
            
        # Parameters for numerical optimization
        max_iterations = 1000
        tolerance = 1e-2
        damping = 0.5
        
        # Solve numerically using Jacobian
        for iteration in range(max_iterations):
            # Set current joint positions
            self.set_joint_positions(q)
            
            # Visualize intermediate steps if requested
            if visualize_steps and self.visualize and self.meshcat and (iteration % 20 == 0):
                # Show intermediate pose
                self.forward_kinematics(q, visualize=True)
                # Allow the visualization to update
                time.sleep(0.05)
            
            # Get current end effector pose
            current_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
            
            # Calculate position error
            pos_error = target_pose.translation() - current_pose.translation()
            
            # Simple orientation error based on rotation matrix
            R_current = current_pose.rotation().matrix()
            R_target = target_pose.rotation().matrix()
            R_error = R_target @ R_current.T
            orientation_error = np.zeros(3)
            orientation_error[0] = R_error[2, 1] - R_error[1, 2]
            orientation_error[1] = R_error[0, 2] - R_error[2, 0]
            orientation_error[2] = R_error[1, 0] - R_error[0, 1]
            
            # Combine errors into a 6D error vector
            error = np.hstack([pos_error, 0.5 * orientation_error])
            error_norm = np.linalg.norm(error)
            
            # Check convergence
            if error_norm < tolerance:
                break
                
            # Calculate Jacobian
            J = np.zeros((6, len(self.joint_indices)))
            for i in range(len(self.joint_indices)):
                # Finite difference approximation for Jacobian
                eps = 1e-6
                q_perturbed = q.copy()
                q_perturbed[i] += eps
                
                self.set_joint_positions(q_perturbed)
                perturbed_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
                
                # Position part
                J[0:3, i] = (perturbed_pose.translation() - current_pose.translation()) / eps
                
                # Rotation part - simpler approximation
                R_perturbed = perturbed_pose.rotation().matrix()
                R_diff = R_perturbed @ R_current.T
                rot_diff = np.zeros(3)
                rot_diff[0] = R_diff[2, 1] - R_diff[1, 2]
                rot_diff[1] = R_diff[0, 2] - R_diff[2, 0]
                rot_diff[2] = R_diff[1, 0] - R_diff[0, 1]
                J[3:6, i] = rot_diff / eps
            
            # Reset to original pose for this iteration
            self.set_joint_positions(q)
            
            # Damped least squares for stability
            J_dag = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(6))
            
            # Update joint positions
            dq = J_dag @ error
            q = q + dq
            
            # Apply joint limits
            for i in range(len(self.joint_indices)):
                q[i] = max(min(q[i], self.joint_upper_limits[i]), self.joint_lower_limits[i])
        
        # Final evaluation of the solution
        self.set_joint_positions(q)
        final_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
        final_pos_error = np.linalg.norm(target_pose.translation() - final_pose.translation())
        
        # Calculate rotation error for reporting
        final_R = final_pose.rotation().matrix()
        target_R = target_pose.rotation().matrix()
        R_error_final = final_R.T @ target_R
        angle_axis_final = RotationMatrix(R_error_final).ToAngleAxis()
        rot_error_final = angle_axis_final.angle()
        
        # Determine success with a slightly relaxed tolerance
        success = final_pos_error < tolerance * 10
        
        # Check self collisions first
        self_collisions = self.check_self_collisions()
        
        # Now check point cloud collisions with our custom method
        point_cloud_collisions = self.check_point_cloud_collisions()
        
        # Print point cloud collisions for debugging
        if point_cloud_collisions:
            print(f"Found {len(point_cloud_collisions)} point cloud collisions:")
            for col in point_cloud_collisions:
                print(f"  Collision between robot link '{col['body_name']}' and point cloud point {col['point_index']}")
                print(f"    Penetration depth: {col['depth']:.6f}")
                print(f"    Link position: {col['link_pos']}")
                print(f"    Point position: {col['point_pos']}")
        
        # Return results
        info = {
            "success": success,
            "iterations": iteration + 1,
            "position_error": final_pos_error,
            "rotation_error": rot_error_final,
            "collision_free": len(self_collisions) == 0 and len(point_cloud_collisions) == 0,
            "num_collisions": len(self_collisions),
            "point_cloud_collisions": len(point_cloud_collisions)
        }
        
        return q, info
        
        # If we reached max iterations, return the best solution so far
        self.set_joint_positions(q)
        collisions = self.check_self_collisions()
        
        # Count point cloud collisions
        point_cloud_collisions = 0
        if hasattr(self, 'point_cloud_geometries') and self.point_cloud_geometries and len(self.point_cloud_geometries) > 0:
            # Extract just the string representations for comparison
            pc_geom_id_strings = [pc_id[1] for pc_id in self.point_cloud_geometries]
            
            for col in collisions:
                geometry_A_id = str(col.id_A)
                geometry_B_id = str(col.id_B)
                
                # Check if either geometry is part of the point cloud
                is_point_cloud_collision = False
                for pc_geom_id_str in pc_geom_id_strings:
                    # Check for exact match rather than substring
                    if pc_geom_id_str == geometry_A_id or pc_geom_id_str == geometry_B_id:
                        point_cloud_collisions += 1
                        is_point_cloud_collision = True
                        break
                
                if is_point_cloud_collision:
                    print(f"Found point cloud collision: {geometry_A_id} with {geometry_B_id}, depth={col.depth}")
        
        # Calculate final errors
        current_pose = self.forward_kinematics(q)
        pos_error = target_pose.translation() - current_pose.translation()
        pos_error_norm = np.linalg.norm(pos_error)
        
        current_R = current_pose.rotation().matrix()
        target_R = target_pose.rotation().matrix()
        R_error = current_R.T @ target_R
        angle_axis = RotationMatrix(R_error).ToAngleAxis()
        rot_error_norm = angle_axis.angle()
        
        return q, {
            "success": False,  # Did not converge within max iterations
            "iterations": max_iterations,
            "position_error": pos_error_norm,
            "rotation_error": rot_error_norm,
            "collision_free": len(collisions) == 0,
            "num_collisions": len(collisions),
            "point_cloud_collisions": point_cloud_collisions
        }
        
    def _eval_position_constraint(self, q, target_pose, axis):
        """Evaluate the position constraint along a specific axis."""
        self.set_joint_positions(q)
        current_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
        
        # Calculate position error along the specified axis
        pos_error = current_pose.translation()[axis] - target_pose.translation()[axis]
        return [pos_error]
    
    def _eval_orientation_constraint(self, q, target_pose, axis):
        """Evaluate the orientation constraint along a specific axis."""
        self.set_joint_positions(q)
        current_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
        
        # Calculate orientation error
        # We'll use a simplified approach based on the rotation matrix
        current_R = current_pose.rotation().matrix()
        target_R = target_pose.rotation().matrix()
        
        # Extract the main axes and compute their alignment
        current_axis = current_R[:, axis]
        target_axis = target_R[:, axis]
        
        # Dot product measures alignment - we want this to be 1
        alignment = np.dot(current_axis, target_axis)
        
        # We want alignment to be 1, so error is 1 - alignment
        return [1.0 - alignment]

def publish_joint_state_to_lcm(joint_names, joint_positions):
    """Publish joint positions to LCM.
    
    Args:
        joint_names: List of joint names
        joint_positions: List of joint positions matching the names
    """
    # Initialize LCM
    lc = lcm.LCM()
    
    # Create JointState message
    msg = JointState()
    
    # Check if the message has a header and handle appropriately
    if hasattr(msg, 'header'):
        try:
            # If std_msgs is available and used, initialize header
            import std_msgs
            msg.header = std_msgs.Header()
        except (ImportError, AttributeError):
            # If header exists but we can't initialize it properly, set it to None
            # This might be okay as some LCM implementations don't require header
            pass
    
    # Set message fields
    msg.name = joint_names
    msg.position = joint_positions
    msg.velocity = []  # Empty list by default
    msg.effort = []    # Empty list by default
    msg.name_length = len(msg.name)
    msg.position_length = len(msg.position)
    msg.velocity_length = len(msg.velocity)
    msg.effort_length = len(msg.effort)
    
    # Publish the message
    lc.publish("joint_states#sensor_msgs.JointState", msg.encode())
    print("\nPublished joint state to LCM topic 'joint_states#sensor_msgs.JointState'")

def load_and_downsample_pointcloud(file_path, voxel_size=None, sample_ratio=None):
    """Load and optionally downsample a point cloud.
    
    Args:
        file_path: Path to the point cloud file (.pcd, .ply, etc.)
        voxel_size: If provided, downsample using voxel grid filtering with this voxel size
        sample_ratio: If provided, randomly sample this ratio of points (0.0-1.0)
        
    Returns:
        Downsampled point cloud as numpy array of shape (N, 3)
    """
    if not os.path.exists(file_path):
        print(f"Point cloud file not found: {file_path}")
        return None
    
    # Load the point cloud
    print(f"Loading point cloud from {file_path}")
    if file_path.endswith('.pcd'):
        pcd = o3d.io.read_point_cloud(file_path)
    elif file_path.endswith('.ply'):
        pcd = o3d.io.read_triangle_mesh(file_path).sample_points_uniformly(number_of_points=100000)
    else:
        print(f"Unsupported point cloud format: {file_path}")
        return None
    
    original_size = len(np.asarray(pcd.points))
    print(f"Original point cloud has {original_size} points")
    
    # Downsample using voxel grid if specified
    if voxel_size is not None and voxel_size > 0:
        print(f"Downsampling with voxel grid of size {voxel_size}")
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # Random sampling if specified
    if sample_ratio is not None and 0.0 < sample_ratio < 1.0:
        points = np.asarray(pcd.points)
        num_points = len(points)
        sample_size = max(1, int(num_points * sample_ratio))
        print(f"Random sampling {sample_ratio:.2f} of points ({sample_size} points)")
        indices = np.random.choice(num_points, sample_size, replace=False)
        pcd = pcd.select_by_index(indices)
    
    # Convert to numpy array
    points = np.asarray(pcd.points)
    print(f"Downsampled point cloud has {len(points)} points")
    return points

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Inverse Kinematics with Collision Checking')
    parser.add_argument('--urdf', type=str, default="../assets/devkit_base_descr.urdf",
                        help='Path to the URDF file')
    parser.add_argument('--point-cloud', type=str, help='Path to point cloud file (.pcd or .ply)')
    parser.add_argument('--voxel-size', type=float, default=0.01,
                        help='Voxel size for downsampling the point cloud')
    parser.add_argument('--sample-ratio', type=float, default=1.0,
                        help='Ratio for random sampling the point cloud (0.0-1.0)')
    parser.add_argument('--point-radius', type=float, default=0.01,
                        help='Radius of point cloud points for collision checking (in meters)')
    parser.add_argument('--no-visualize', action='store_true',
                        help='Disable 3D visualization')
    parser.add_argument('--visualize-steps', action='store_true',
                        help='Visualize intermediate IK steps')
    parser.add_argument('--test-collision', action='store_true',
                        help='Run an additional test to verify point cloud collision detection')
    args = parser.parse_args()
    
    # Path to the URDF file
    urdf_path = os.path.abspath(args.urdf)
    
    # Define the kinematic chain and end effector
    kinematic_chain_joints = ["pillar_platform_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    end_effector_link = "link6"
    
    # Load and downsample point cloud if specified
    point_cloud = None
    if args.point_cloud:
        point_cloud = load_and_downsample_pointcloud(
            args.point_cloud, 
            voxel_size=args.voxel_size,
            sample_ratio=args.sample_ratio
        )
        print(f"Using point cloud radius of {args.point_radius} meters for collision detection")
    
    # Create the IK solver
    ik_solver = URDFInverseKinematicsWithCollisionChecker(
        urdf_path, end_effector_link, kinematic_chain_joints,
        point_cloud=point_cloud, point_cloud_radius=args.point_radius,
        visualize=not args.no_visualize
    )
    
    # Run calibration to identify links that are frequently in collision
    ik_solver.run_calibration(num_samples=10000, collision_threshold=0.95)
    
    # Test IK by first running forward kinematics
    print("\nTesting IK with FK validation:")
    
    # Create a test joint configuration
    test_q = np.zeros(len(ik_solver.joint_indices))
    # for i in range(len(ik_solver.joint_indices)):
    #     # Set to middle of joint range
    #     test_q[i] = (ik_solver.joint_lower_limits[i] + ik_solver.joint_upper_limits[i]) / 2.0
    
    # Print initial joint positions
    print("\nInitial Joint Positions:")
    joint_positions = ik_solver.get_joint_name_positions(test_q)
    for joint_name, position in joint_positions.items():
        print(f"  {joint_name}: {position:.6f} radians")
            
    # Run forward kinematics to get end effector pose and visualize initial configuration
    ee_pose = ik_solver.forward_kinematics(test_q, visualize=True)
    if ik_solver.visualize and ik_solver.meshcat:
        # Create a blue sphere to mark the initial position
        blue = np.array([0.0, 0.0, 1.0, 1.0])  # RGBA format
        ik_solver.meshcat.SetObject("/markers/initial", 
                               Sphere(0.03),  # Small sphere
                               Rgba(blue[0], blue[1], blue[2], blue[3]))
        # Position the marker at the initial end effector location
        ik_solver.meshcat.SetTransform("/markers/initial", ee_pose)
        # Give user time to view
        time.sleep(1.0)
    # print(f"\nFK test pose:\n{ee_pose.GetAsMatrix4()}")
    print(f"\nFK test pose:\n{ee_pose.translation()}\n{ee_pose.rotation()}")
            
    # Check if initial configuration has collisions
    collisions = ik_solver.get_collision_geometry_names()
    if collisions:
        print("\nInitial configuration has collisions:")
        for collision in collisions:
            print(f"  Collision between {collision['geometry_A']} (Link: {collision['link_A']})")
            print(f"    and {collision['geometry_B']} (Link: {collision['link_B']})")
            print(f"    Penetration depth: {collision['depth']:.6f}")
            print(f"    Locations: {collision['frame_A']} and {collision['frame_B']}")
    else:
        print("\nInitial configuration is collision-free")
    
    # set rotation based on euler angles
    target_rotation = RotationMatrix(RollPitchYaw([np.pi/2, 0.0, np.pi/2]))
    target_pose = RigidTransform(
        target_rotation,
        ee_pose.translation() - np.array([-0.15, 0.15, 1.12])  # Match the original script target
    )
    # target_pose = RigidTransform(
    #     ee_pose.rotation(),
    #     ee_pose.translation() - np.array([0.3, 0.0, 0.8])  # Match the original script target
    # )
    print(f"\nTarget pose for IK:\n{target_pose.translation()}\n{target_pose.rotation()}")
    
    # Show target pose with a marker in meshcat
    if ik_solver.visualize and ik_solver.meshcat:
        # Clear previous markers (if possible with this API version)
        try:
            ik_solver.meshcat.Delete("/markers")
        except:
            pass
        
        # Create a green sphere for the target position
        green = np.array([0.0, 1.0, 0.0, 1.0])  # RGBA format
        ik_solver.meshcat.SetObject("/markers/target", 
                               Sphere(0.05),  # Sphere
                               Rgba(green[0], green[1], green[2], green[3]))
        # Position the marker at the target location
        ik_solver.meshcat.SetTransform("/markers/target", target_pose)
        # Give user time to view
        time.sleep(1.0)
        
    # Run inverse kinematics to solve for the target pose
    start_time = time.time()
    q_sol, info = ik_solver.inverse_kinematics(target_pose, initial_guess=test_q, 
                                         visualize_steps=args.visualize_steps)
    end_time = time.time()
    
    # Print results
    print(f"\nIK solved in {end_time - start_time:.4f} seconds")
    print(f"Solution info: {info}")
    
    if True:
        print("\nIK solution found!")
        
        # Print solution joint positions
        print("\nSolution Joint Positions:")
        solution_joint_positions = ik_solver.get_joint_name_positions(q_sol)
        for joint_name, position in solution_joint_positions.items():
            initial_pos = joint_positions.get(joint_name, 0.0)
            delta = position - initial_pos
            print(f"  {joint_name}: {position:.6f} radians (change: {delta:+.6f})")
        
        # Validate solution with forward kinematics and visualize
        result_pose = ik_solver.forward_kinematics(q_sol, visualize=True)
        if ik_solver.visualize and ik_solver.meshcat:
            # Mark the solution with a red sphere
            red = np.array([1.0, 0.0, 0.0, 1.0])  # RGBA format
            ik_solver.meshcat.SetObject("/markers/solution", 
                                   Sphere(0.04),  # Small sphere
                                   Rgba(red[0], red[1], red[2], red[3]))
            # Position the marker at the solution end effector location
            ik_solver.meshcat.SetTransform("/markers/solution", result_pose)
        print(f"\nResult pose from IK solution:\n{result_pose.translation()}\n{result_pose.rotation()}")
        
        # Calculate position error
        pos_error = np.linalg.norm(result_pose.translation() - target_pose.translation())
        rot_error = RotationMatrix(result_pose.rotation().matrix().T @ target_pose.rotation().matrix()).ToAngleAxis().angle()
        
        print(f"\nPosition error: {pos_error:.6f} meters")
        print(f"Rotation error: {rot_error:.6f} radians")
        
        # Report point cloud collisions if available
        if "point_cloud_collisions" in info and info["point_cloud_collisions"] > 0:
            print(f"\nSolution has {info['point_cloud_collisions']} collisions with the point cloud")
        elif ik_solver.point_cloud is not None:
            print("\nSolution has no collisions with the point cloud")
        
        # Check for collisions in the solution
        collisions = ik_solver.get_collision_geometry_names()
        if collisions:
            print("\nSolution configuration has collisions:")
            for collision in collisions:
                print(f"  Collision between {collision['geometry_A']} (Link: {collision['link_A']})")
                print(f"    and {collision['geometry_B']} (Link: {collision['link_B']})")
                # print(f"    Penetration depth: {collision['depth']:.6f}")
                print(f"    Locations: {collision['frame_A']} and {collision['frame_B']}")
            
            # Visualize the collision points in red
            ik_solver.visualize_collision_points(collisions)
        else:
            print("\nSolution configuration is collision-free")
        
        # Publish joint positions to LCM for visualization
        joint_names = list(solution_joint_positions.keys())
        joint_values = [solution_joint_positions[name] for name in joint_names]
        publish_joint_state_to_lcm(joint_names, joint_values)
        
        print("\nJoint state published to LCM for visualization")
        
        # Perform a deliberate collision test to verify that point cloud detection works
        if ik_solver.point_cloud is not None and len(ik_solver.point_cloud) > 0:
            print("\nRunning deliberate collision test to verify detection...")
            
            # Get centroid of point cloud to aim for
            centroid = np.mean(ik_solver.point_cloud, axis=0)
            print(f"Point cloud centroid: {centroid}")
            
            # Create a test configuration that should deliberately collide with the point cloud
            test_q = np.zeros(len(ik_solver.joint_indices))
            
            # Set each joint to a position that should move the arm into the point cloud
            # This is a heuristic approach that will work for most point clouds
            test_q[0] = 0.0  # Base joint
            test_q[1] = 0.3  # Joint 1
            test_q[2] = 0.6  # Joint 2
            test_q[3] = 0.0  # Joint 3
            test_q[4] = 0.0  # Joint 4
            test_q[5] = 0.0  # Joint 5
            test_q[6] = 0.0  # Joint 6
            
            # Set the test configuration
            ik_solver.set_joint_positions(test_q)
            
            # Visualize this configuration
            ik_solver.forward_kinematics(test_q, visualize=True)
            
            # Check for point cloud collisions
            print("Checking for collisions in test configuration...")
            pc_collisions = ik_solver.check_point_cloud_collisions()
            
            if pc_collisions:
                print(f"✓ SUCCESS! Detected {len(pc_collisions)} point cloud collisions in test configuration")
                for col in pc_collisions[:3]:  # Show first 3 collisions
                    print(f"  Collision between robot link '{col['body_name']}' and point cloud point {col['point_index']}")
                    print(f"    Penetration depth: {col['depth']:.6f} meters")
                print("Point cloud collision detection is working correctly!")
            else:
                print("× FAILURE: No point cloud collisions detected in test configuration.")
                
                # Try a more aggressive collision position
                print("Trying a more aggressive collision position...")
                
                # Try to position a link directly at the centroid of the point cloud
                for i in range(len(ik_solver.joint_indices)):
                    test_q[i] = 0.5 * ik_solver.joint_upper_limits[i]  # Use 50% of the joint range
                
                # Move the pillar joint to get closer to the point cloud
                test_q[0] = -0.3  # Typically moves closer to the cloud
                
                # Set and visualize
                ik_solver.set_joint_positions(test_q)
                ik_solver.forward_kinematics(test_q, visualize=True)
                
                # Check again
                pc_collisions = ik_solver.check_point_cloud_collisions()
                
                if pc_collisions:
                    print(f"✓ SUCCESS! Detected {len(pc_collisions)} point cloud collisions in second test")
                    print("Point cloud collision detection is working correctly!")
                else:
                    print("× FAILURE: Still no collisions detected. Further debugging needed.")
                    print("Try increasing the point_radius parameter for easier collision detection.")
    
    print("KEEPING MESHCAT VISUALIZATION OPEN. Press Ctrl+C to exit when finished viewing.")
    try:
        print("Meshcat visualization running... Press Ctrl+C to exit")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting visualization...")

if __name__ == "__main__":
    main()

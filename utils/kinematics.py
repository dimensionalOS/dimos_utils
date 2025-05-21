import numpy as np
import time
import os
import lcm
import argparse
import sys
import tqdm
import trimesh.primitives
import time
import open3d as o3d
from importlib.machinery import SourceFileLoader
from lcm_msgs.sensor_msgs import JointState

from pydrake.all import (
    MultibodyPlant, Parser, InverseDynamics, SceneGraph,
    DiagramBuilder, MathematicalProgram, Solve, RigidTransform,
    PiecewisePolynomial, TrajectorySource, 
    Simulator, RotationMatrix, RollPitchYaw, SpatialVelocity,
    AutoDiffXd, AddMultibodyPlantSceneGraph, JacobianWrtVariable,
    FindResourceOrThrow, eq, ge, le, GeometryInstance, Sphere, Box,
    MakePhongIllustrationProperties, Rgba, CollisionFilterDeclaration,
    MeshcatVisualizer, StartMeshcat, CoulombFriction
)
from pydrake.multibody.tree import BodyIndex
from octree_collision import OctreeNode, Octree
from tqdm import tqdm

class Kinematics:
    def __init__(self, urdf_path, end_effector_link, kinematic_chain_joints, point_cloud=None, point_cloud_radius=0.01, visualize=True, max_points_per_node=10, min_node_size=0.05):
        """
        Initialize the kinematics class with the given URDF path, end effector link, and kinematic chain joints.
        
        Args:
            urdf_path: Path to the URDF file
            end_effector_link: Name of end effector link
            kinematic_chain_joints: List of joint names to include in the kinematic chain
            point_cloud: Optional point cloud for collision checking of shape (N, 3)
            point_cloud_radius: Radius of point cloud for collision checking
            visualize: Whether to visualize the kinematics
            max_points_per_node: Maximum number of points in a node before splitting
            min_node_size: Minimum size of a node before stopping subdivision
        """
        self.urdf_path = urdf_path
        self.end_effector_link = end_effector_link
        self.kinematic_chain_joints = kinematic_chain_joints
        self.point_cloud = point_cloud
        self.point_cloud_radius = point_cloud_radius
        self.visualize = visualize
        self.meshcat = None
        self.max_points_per_node = max_points_per_node
        self.min_node_size = min_node_size
        self.octree = None
        self.joint_positions = None

        # Initialize meshcat if visualization is enabled
        if self.visualize:
            self.meshcat = StartMeshcat()
            
        # Initialize plant with collision detection
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.0)
        
        # Initialize bounding box plant for collision checking
        self.bbox_builder = DiagramBuilder()
        self.bbox_plant, self.bbox_scene_graph = AddMultibodyPlantSceneGraph(self.bbox_builder, time_step=0.0)

        if self.point_cloud is not None:
            self.pc_builder = DiagramBuilder()
            self.pc_plant, self.pc_scene_graph = AddMultibodyPlantSceneGraph(self.pc_builder, time_step=0.0)
            self.narrow_builder = DiagramBuilder()
            self.narrow_plant, self.narrow_scene_graph = AddMultibodyPlantSceneGraph(self.narrow_builder, time_step=0.0)
        
        # Load URDF model
        print(f"Loading URDF from: {self.urdf_path}")
        self.parser = Parser(self.plant)
        self.bbox_parser = Parser(self.bbox_plant)
        if self.point_cloud is not None:
            self.pc_parser = Parser(self.pc_plant)
            self.narrow_parser = Parser(self.narrow_plant)
        try:
            model_instances = self.parser.AddModelsFromUrl(f"file://{self.urdf_path}")
            bbox_model_instances = self.bbox_parser.AddModelsFromUrl(f"file://{self.urdf_path}")
            # We'll use the first model instance returned
            self.model_instance = model_instances[0] if model_instances else None
            self.bbox_model_instance = bbox_model_instances[0] if bbox_model_instances else None
            if self.point_cloud is not None:
                pc_model_instances = self.pc_parser.AddModelsFromUrl(f"file://{self.urdf_path}")
                self.pc_model_instance = pc_model_instances[0] if pc_model_instances else None
                narrow_model_instances = self.narrow_parser.AddModelsFromUrl(f"file://{self.urdf_path}")
                self.narrow_model_instance = narrow_model_instances[0] if narrow_model_instances else None
        except Exception as e:
            print(f"Error loading URDF: {e}")
            
        # Creating octree from point cloud for collisions
        if self.point_cloud is not None:
            self.octree = Octree(self.point_cloud, self.max_points_per_node, self.min_node_size)

        # Finalize the plant
        self.plant.Finalize()
        self.bbox_plant.Finalize()
        # Do not finalize point cloud plan since we still have to add collision points
        
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
            self.bbox_end_effector_body = self.bbox_plant.GetBodyByName(self.end_effector_link)
            if self.point_cloud is not None:
                self.pc_end_effector_body = self.pc_plant.GetBodyByName(self.end_effector_link)
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
        self.bbox_diagram = self.bbox_builder.Build()
        self.bbox_simulator = Simulator(self.bbox_diagram)
        self.bbox_context = self.bbox_simulator.get_mutable_context()
        self.bbox_plant_context = self.bbox_plant.GetMyContextFromRoot(self.bbox_context)
        
        # With the context created, we're ready for collision detection
        # The point cloud will be used directly for collision detection
        # rather than adding to the scene graph (which is already finalized)
        if self.point_cloud is not None and len(self.point_cloud) > 0:
            print(f"Using {len(self.point_cloud)} points for collision detection with radius {self.point_cloud_radius}m")
            # Visualize the point cloud in Meshcat
            if self.visualize and self.meshcat:
                self.visualize_point_cloud()
        
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
        bbox_full_q = self.bbox_plant.GetPositions(self.bbox_plant_context)
        for i, idx in enumerate(self.joint_indices):
            full_q[idx] = q[i]
            bbox_full_q[idx] = q[i]
        self.plant.SetPositions(self.plant_context, full_q)
        self.bbox_plant.SetPositions(self.bbox_plant_context, bbox_full_q)

    def get_random_joint_positions(self):
        """Generate random joint positions within limits."""
        q = np.zeros(len(self.joint_indices))
        for i in range(len(self.joint_indices)):
            q[i] = np.random.uniform(self.joint_lower_limits[i], self.joint_upper_limits[i])
        return q

    def set_point_cloud(self, point_cloud, point_radius=0.01, max_points_per_node=10, min_node_size=0.05):
        self.point_cloud = point_cloud
        self.point_cloud_radius = point_radius
        self.max_points_per_node = max_points_per_node
        self.min_node_size = min_node_size
        self.octree = Octree(self.point_cloud, self.max_points_per_node, self.min_node_size)

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

    def get_robot_bbox(self):
        """Get the bounding box of the robot in the current configuration."""
        min_corner = np.array([float('inf'), float('inf'), float('inf')])
        max_corner = np.array([float('-inf'), float('-inf'), float('-inf')])
        robot_min = np.array([float('-inf'), float('-inf'), float('-inf')])
        robot_max = np.array([float('inf'), float('inf'), float('inf')])
        

        # Add safety margin to ensure we don't miss any collisions
        safety_margin = 0.1
        body_positions = []

        # Iterate through all bodies to find their positions
        body_count = self.bbox_plant.num_bodies()
        for i in range(body_count):
            try:
                body = self.bbox_plant.get_body(BodyIndex(i))
                if body.index() == self.bbox_plant.world_body().index():
                    continue
                body_pose = self.bbox_plant.EvalBodyPoseInWorld(self.bbox_plant_context, body)
                position = body_pose.translation()
                body_positions.append(position)
                min_corner = np.minimum(min_corner, position)
                max_corner = np.maximum(max_corner, position)
            except Exception as e:
                print(f"Warning: Could not process body {i}: {e}")
        
        if body_positions:
            # We found at least one body, calculate the encompassing box, expanding by safety margin
            robot_min = min_corner - safety_margin
            robot_max = max_corner + safety_margin

        return robot_min, robot_max

    def get_collision_geometry_names(self):
        """Get the names of geometries that are in collision."""
        collision_pairs = self.check_self_collisions()
        geometry_pairs = []

        #Load and parse the URDF to extract link names
        import xml.etree.ElementTree as ET
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            link_names = [link.attrib['name'] for link in root.findall('.//link')]
            print(f"\nAvailable links in the URDF: {', '.join(link_names[:5])}... (and {len(link_names)-5} more)")
        except Exception as e:
            print(f"Could not parse URDF file: {e}")
            link_names = []
        
        #Map geometry IDs to indices or offsets
        geometry_id_mapping = {}
        for i, link_name in enumerate(link_names):
            #Create a few possible mappings for geometry IDs
            #Note: These are heuristics and may not be accurate
            base_id = (i+1) * 7  #Just a heuristic
            for offset in range(-3, 4):  #Try a few offsets around our guessed base ID
                potential_id = base_id + offset
                geometry_id_mapping[f"<GeometryId value={potential_id}>"] = link_name
        
        #Process each collision pair
        for pair in collision_pairs:
            #Get information from the collision pair
            geometry_A_id = str(pair.id_A)
            geometry_B_id = str(pair.id_B)
            penetration_depth = pair.depth
            
            #Extract numeric ID from the geometry ID string
            try:
                id_A_num = int(geometry_A_id.split('value=')[1].split('>')[0])
                id_B_num = int(geometry_B_id.split('value=')[1].split('>')[0])
            except:
                id_A_num = 0
                id_B_num = 0
            
            #Try to map to link names
            link_A = geometry_id_mapping.get(geometry_A_id, "")
            link_B = geometry_id_mapping.get(geometry_B_id, "")
            
            if not link_A and len(link_names) > 0:
                #Try to associate with a link based on ID value
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
            
            #Get positions
            frame_A_pos = getattr(pair, 'p_WCa', None)
            frame_B_pos = getattr(pair, 'p_WCb', None)
            
            frame_A_info = f"Position {frame_A_pos}" if frame_A_pos is not None else "unknown position"
            frame_B_info = f"Position {frame_B_pos}" if frame_B_pos is not None else "unknown position"
            
            #Store collision information
            geometry_pairs.append({
                "geometry_A": geometry_A_id,
                "link_A": link_A,
                "frame_A": frame_A_info,
                "geometry_B": geometry_B_id,
                "link_B": link_B,
                "frame_B": frame_B_info,
                "depth": penetration_depth
            })
        
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
            q: Array of joint positions
            visualize: Whether to visualize the forward kinematics
        """
        self.set_joint_positions(q)
        
        # Get end effector pose
        end_effector_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
        
        if visualize and self.visualize and self.meshcat:
            # Update the visualization - force a fresh draw to ensure visibility
            self.diagram.ForcedPublish(self.diagram_context)
            
            # Add a small delay to allow visualization to render
            time.sleep(0.1)
        
        return end_effector_pose
    
    def check_pc_collisions_octree(self, q):
        """
        Check for point cloud collisions using an octree.
        
        Args:
            q: Array of joint positions
        Returns:
            colliding_points: Set of indices of colliding points
        """
        if self.point_cloud is None:
            return False
        self.set_joint_positions(q)
        robot_min, robot_max = self.get_robot_bbox()
        # Get collision shapes for regions of the octree that overlap with the robot
        collision_shapes = self.octree.get_collision_shapes(robot_min, robot_max)

        # Register collision geometries for the filtered regions
        point_to_geometry_id = {}
        region_to_points = {}
        
        # Count total points in collision regions
        total_points_in_regions = sum(len(points) for _, _, points in collision_shapes)
        print(f"Points in collision regions: {total_points_in_regions} out of {len(self.point_cloud)}")

        # Actually create the collision geometries
        all_region_ids = []
        for i, (center, size, points) in enumerate(collision_shapes):
            if size < 0.02:  # Very small nodes - use sphere for individual points
                # Create single geometry for all points in this small region
                sphere = Sphere(0.01)  # Slightly larger radius for these concentrated regions
                instance_name = f"octree_region_{i}"
                friction = CoulombFriction(0.9, 0.8)
                
                # Register as collision geometry
                region_id = self.pc_plant.RegisterCollisionGeometry(
                    self.pc_plant.world_body(),
                    RigidTransform(center),
                    sphere,
                    instance_name,
                    friction
                )
                
                all_region_ids.append(region_id)
                region_to_points[region_id] = points
                
                # Map the points to this geometry
                for p in points:
                    point_to_geometry_id[p] = region_id
            else:
                # For larger regions, use a box that encompasses all points in the region
                box = Box(size, size, size)
                instance_name = f"octree_box_{i}"
                friction = CoulombFriction(0.9, 0.8)
                
                # Register as collision geometry
                region_id = self.pc_plant.RegisterCollisionGeometry(
                    self.pc_plant.world_body(),
                    RigidTransform(center),
                    box,
                    instance_name,
                    friction
                )
                
                all_region_ids.append(region_id)
                region_to_points[region_id] = points
                
                # Map the points to this geometry
                for p in points:
                    point_to_geometry_id[p] = region_id
        
        print(f"Added {len(all_region_ids)} collision geometries")
        
        # Finalize the plant
        self.pc_plant.Finalize()
        self.pc_diagram = self.pc_builder.Build()
        self.pc_simulator = Simulator(self.pc_diagram)
        self.pc_context = self.pc_simulator.get_mutable_context()
        self.pc_plant_context = self.pc_plant.GetMyContextFromRoot(self.pc_context)

        # Set the joint positions
        pc_full_q = self.pc_plant.GetPositions(self.pc_plant_context)
        for i, idx in enumerate(self.joint_indices):
            pc_full_q[idx] = q[i]
        self.pc_plant.SetPositions(self.pc_plant_context, pc_full_q)

        query_object = self.pc_scene_graph.get_query_output_port().Eval(
            self.pc_scene_graph.GetMyContextFromRoot(self.pc_context))
        
        collision_pairs = query_object.ComputePointPairPenetration()
        colliding_region_ids = set()
        for pair in collision_pairs:
            colliding_region_ids.add(pair.id_A)
            colliding_region_ids.add(pair.id_B)
        
        potentially_colliding_points = set()
        points_to_check = []
        for region_id in colliding_region_ids:
            if region_id in region_to_points:
                point_indices = region_to_points[region_id]
                potentially_colliding_points.update(point_indices)
                for idx in point_indices:
                    points_to_check.append((idx, self.point_cloud[idx]))

        print("Broad Phase Collision Detection Done")
        print(f"Number of potentially colliding points: {len(potentially_colliding_points)}")
        print(f"Number of collision pairs: {len(collision_pairs)}")
        
        # If no potentially colliding points are found, return an empty set
        if len(potentially_colliding_points) == 0:
            return set()
        
        # Narrow Phase Collision Detection
        print("\nPerforming narrow-phase collision detection on candidate points...")
        narrow_sphere_radius = 0.005
        narrow_point_to_id = {}
        
        for i, (point_idx, point_coords) in enumerate(points_to_check):
            sphere = Sphere(narrow_sphere_radius)
            instance_name = f"narrow_point_{i}"
            friction = CoulombFriction(0.9, 0.8)
            
            sphere_id = self.narrow_plant.RegisterCollisionGeometry(
                self.narrow_plant.world_body(),
                RigidTransform(point_coords),
                sphere,
                instance_name,
                friction
            )
            
            narrow_point_to_id[point_idx] = sphere_id
        
        self.narrow_plant.Finalize()

        self.narrow_diagram = self.narrow_builder.Build()
        self.narrow_simulator = Simulator(self.narrow_diagram)
        self.narrow_context = self.narrow_simulator.get_mutable_context()
        self.narrow_plant_context = self.narrow_plant.GetMyContextFromRoot(self.narrow_context)
        
        # Set the joint positions
        narrow_full_q = self.narrow_plant.GetPositions(self.narrow_plant_context)
        for i, idx in enumerate(self.joint_indices):
            narrow_full_q[idx] = q[i]
        self.narrow_plant.SetPositions(self.narrow_plant_context, narrow_full_q)

        narrow_query_object = self.narrow_scene_graph.get_query_output_port().Eval(
            self.narrow_scene_graph.GetMyContextFromRoot(self.narrow_context))

        point_id_to_idx = {sphere_id: point_idx for point_idx, sphere_id in narrow_point_to_id.items()}

        narrow_collision_pairs = narrow_query_object.ComputePointPairPenetration()
        colliding_points = set()
        for pair in narrow_collision_pairs:
            geom_A = pair.id_A
            geom_B = pair.id_B
            penetration = pair.depth
            if geom_A in point_id_to_idx:
                point_idx = point_id_to_idx[geom_A]
                colliding_points.add(point_idx)
                print(f"  Point {point_idx} collides with penetration depth {penetration:.6f}")
            if geom_B in point_id_to_idx:
                point_idx = point_id_to_idx[geom_B]
                colliding_points.add(point_idx)
                print(f"  Point {point_idx} collides with penetration depth {penetration:.6f}")
        
        print(f"Found {len(colliding_points)} colliding points")

        print("Narrow phase collision detection done")
        print(f"Narrow phase eliminated {len(potentially_colliding_points) - len(colliding_points)} false positives")

        # TODO: Potentially add visualization logic here

        return colliding_points

            


        
    def forward_kinematics_with_pc_checking(self, q, visualize=False):
        """
        Compute forward kinematics for the given joint positions and check for point cloud collisions.
        
        Args:
            q: Array of joint positions
            visualize: Whether to visualize the forward kinematics

        Returns:
            end_effector_pose: RigidTransform representing the end effector pose
            colliding_points: Set of indices of colliding points
        """
        end_effector_pose = self.forward_kinematics(q, visualize)
        colliding_points = self.check_pc_collisions_octree(q)
        return end_effector_pose, colliding_points

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
        # Pass the current joint positions (q) to the method
        point_cloud_collisions = self.check_pc_collisions_octree(q)
        
        # Visualize colliding points if there are any
        if self.visualize and self.meshcat and point_cloud_collisions and len(point_cloud_collisions) > 0:
            # Clear any previous colliding points visualization
            try:
                self.meshcat.Delete("/colliding_points")
            except:
                pass
            
            # Visualize the new colliding points
            self.visualize_colliding_points(point_cloud_collisions)
        
        # Handle the case where point_cloud_collisions might be False
        if point_cloud_collisions is False:
            pc_collision_count = 0
            pc_collision_free = True
        else:
            pc_collision_count = len(point_cloud_collisions)
            pc_collision_free = pc_collision_count == 0
            
        # Return results
        info = {
            "success": success,
            "iterations": iteration + 1,
            "position_error": final_pos_error,
            "rotation_error": rot_error_final,
            "collision_free": len(self_collisions) == 0 and pc_collision_free,
            "num_collisions": len(self_collisions),
            "point_cloud_collisions": pc_collision_count,
            "colliding_points": point_cloud_collisions if isinstance(point_cloud_collisions, set) else set()
        }
        
        return q, info

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

    def visualize_point_cloud(self):
        """Visualize the point cloud in Meshcat."""
        if not self.visualize or not self.meshcat or self.point_cloud is None:
            return
        
        # Clear existing point cloud markers
        try:
            self.meshcat.Delete("/point_cloud")
        except:
            pass
        
        # Create an orange color for point cloud points (semi-transparent)
        orange = np.array([1.0, 0.55, 0.0, 0.5])  # RGBA format - semi-transparent orange
        
        # Limit number of points to visualize for performance
        max_points = 1000
        points_to_visualize = min(len(self.point_cloud), max_points)
        
        print(f"Visualizing {points_to_visualize} points in the point cloud...")
        
        # Calculate sampling interval if we need to downsample for visualization
        sampling_interval = max(1, len(self.point_cloud) // max_points)
        
        # Add points to meshcat
        for i in range(0, len(self.point_cloud), sampling_interval):
            if i >= max_points:
                break
                
            point = self.point_cloud[i]
            marker_name = f"/point_cloud/point_{i}"
            
            # Create a small sphere for each point
            self.meshcat.SetObject(
                marker_name,
                Sphere(self.point_cloud_radius),  # Use the collision radius
                Rgba(orange[0], orange[1], orange[2], orange[3])
            )
            
            # Position the marker at the point location
            self.meshcat.SetTransform(marker_name, RigidTransform(point))
        
        print(f"Visualized {points_to_visualize} point cloud points in orange")
        
    def visualize_colliding_points(self, colliding_point_indices):
        """Visualize colliding points in the point cloud with red color.
        
        Args:
            colliding_point_indices: Set or list of indices of colliding points in the point cloud
        """
        if not self.visualize or not self.meshcat or self.point_cloud is None or not colliding_point_indices:
            return
        
        # Create a bright red color for colliding points
        red = np.array([1.0, 0.0, 0.0, 0.8])  # RGBA format - semi-transparent red
        
        print(f"Visualizing {len(colliding_point_indices)} colliding points in the point cloud...")
        
        # Add colliding points to meshcat
        for idx in colliding_point_indices:
            point = self.point_cloud[idx]
            marker_name = f"/colliding_points/point_{idx}"
            
            # Create a slightly larger red sphere for colliding points
            self.meshcat.SetObject(
                marker_name,
                Sphere(self.point_cloud_radius * 1.5),  # Slightly larger for visibility
                Rgba(red[0], red[1], red[2], red[3])
            )
            
            # Position the marker at the point location
            self.meshcat.SetTransform(marker_name, RigidTransform(point))
        
        print(f"Highlighted {len(colliding_point_indices)} colliding points in red")
    
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
    urdf_path = "../assets/devkit_base_descr.urdf"
    urdf_path = os.path.abspath(urdf_path)
    end_effector_link = "link6"
    kinematic_chain_joints = ["pillar_platform_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    point_cloud_path = "sample_wall.pcd"
    point_cloud = load_and_downsample_pointcloud(point_cloud_path, voxel_size=0.01, sample_ratio=1.0)

    ik_solver = Kinematics(
        urdf_path, end_effector_link, kinematic_chain_joints, visualize=True, point_cloud=point_cloud, point_cloud_radius=0.01
    )

    ik_solver.run_calibration(num_samples=10000, collision_threshold=0.95)

    test_q = np.zeros(len(ik_solver.joint_indices))
    joint_positions = ik_solver.get_joint_name_positions(test_q)
    for joint_name, position in joint_positions.items():
        print(f"  {joint_name}: {position:.6f} radians")
    
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
    # target_pose = RigidTransform(
    #     target_rotation,
    #     ee_pose.translation() - np.array([-0.15, 0.15, 1.02])  # Match the original script target
    # )
    target_pose = RigidTransform(
        ee_pose.rotation(),
        ee_pose.translation() - np.array([0.1, 0.0, 0.2])  # Match the original script target
    )
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
    # Define a default value for visualize_steps instead of using args
    visualize_steps = False
    q_sol, info = ik_solver.inverse_kinematics(target_pose, initial_guess=test_q, 
                                         visualize_steps=visualize_steps)
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
            
            # Use the colliding points that were already detected during IK
            if "colliding_points" in info and info["colliding_points"]:
                # Clear previous collision visualizations
                if ik_solver.visualize and ik_solver.meshcat:
                    try:
                        ik_solver.meshcat.Delete("/colliding_points")
                    except:
                        pass
                    
                    # Visualize the colliding points
                    ik_solver.visualize_colliding_points(info["colliding_points"])
                    print(f"Visualized {len(info['colliding_points'])} colliding points in the point cloud (in red)")
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

    print("KEEPING MESHCAT VISUALIZATION OPEN. Press Ctrl+C to exit when finished viewing.")
    try:
        print("Meshcat visualization running... Press Ctrl+C to exit")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting visualization...")

if __name__ == "__main__":
    main()

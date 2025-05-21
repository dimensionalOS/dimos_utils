#!/usr/bin/env python3

import numpy as np
import os
import sys
import time
import argparse
import meshcat
from collections import defaultdict

# Import everything from pydrake.all
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    RigidTransform,
    Simulator,
    Sphere,
    Box,
    Cylinder,
    SceneGraph,
    GeometryInstance,
    Role,
    CoulombFriction,
    StartMeshcat,
    MeshcatVisualizer,
    Rgba,
    GeometrySet
)

class OctreeNode:
    """A node in the octree representing a cubic region of space."""
    
    def __init__(self, center, size, max_points=10, min_size=0.05):
        """
        Initialize an octree node.
        
        Args:
            center: Center position of the node's region (3D point)
            size: Size of the node's cubic region
            max_points: Maximum number of points before splitting
            min_size: Minimum size of a node region before stopping subdivision
        """
        self.center = np.array(center)
        self.size = size
        self.max_points = max_points
        self.min_size = min_size
        
        # Bounds of this node's region
        self.min_corner = self.center - self.size/2
        self.max_corner = self.center + self.size/2
        
        # Initially, this is a leaf node with no children
        self.children = None
        self.points = []  # Indices of points in this node
        self.point_coords = []  # Actual coordinates of points
        
    def insert(self, point_idx, point):
        """
        Insert a point into this node.
        
        Args:
            point_idx: Index of the point in the original point cloud
            point: 3D coordinates of the point
        """
        # If this point is not within this node's bounds, ignore it
        if not self._contains_point(point):
            return False
        
        # If we already have children, insert into the appropriate child
        if self.children is not None:
            for child in self.children:
                if child.insert(point_idx, point):
                    return True
            return False  # This should never happen if _contains_point is true
        
        # If we're a leaf node, add the point
        self.points.append(point_idx)
        self.point_coords.append(point)
        
        # Check if we need to split this node
        if len(self.points) > self.max_points and self.size > self.min_size:
            self._split()
            
        return True
    
    def _contains_point(self, point):
        """Check if this node's region contains the given point."""
        return np.all(point >= self.min_corner) and np.all(point <= self.max_corner)
    
    def _split(self):
        """Split this node into 8 children."""
        self.children = []
        
        # Create 8 children (octants)
        new_size = self.size / 2
        offsets = [
            [-1, -1, -1], [+1, -1, -1], [-1, +1, -1], [+1, +1, -1],
            [-1, -1, +1], [+1, -1, +1], [-1, +1, +1], [+1, +1, +1]
        ]
        
        for offset in offsets:
            offset_vector = np.array(offset) * new_size/2
            new_center = self.center + offset_vector
            child = OctreeNode(
                new_center, 
                new_size,
                self.max_points,
                self.min_size
            )
            self.children.append(child)
        
        # Redistribute points to children
        for i, (point_idx, point) in enumerate(zip(self.points, self.point_coords)):
            success = False
            for child in self.children:
                if child.insert(point_idx, point):
                    success = True
                    break
            
            if not success:
                print(f"Warning: Could not insert point {i} into any child node")
        
        # Clear points from this node since they've been redistributed
        self.points = []
        self.point_coords = []
    
    def get_collision_shapes(self, robot_min, robot_max):
        """
        Get collision shapes for this node's region, if it overlaps with the robot.
        
        Args:
            robot_min: Minimum corner of the robot's bounding box
            robot_max: Maximum corner of the robot's bounding box
            
        Returns:
            List of (center, size, points) tuples for collision shapes
        """
        # Check if this node's region overlaps with the robot
        if not self._overlaps_with_box(robot_min, robot_max):
            return []
        
        # If this is a leaf node with points, create a collision shape
        if self.children is None and len(self.points) > 0:
            return [(self.center, self.size, self.points)]
        
        # If this node has children, get shapes from all children
        if self.children is not None:
            shapes = []
            for child in self.children:
                shapes.extend(child.get_collision_shapes(robot_min, robot_max))
            return shapes
        
        # Empty leaf node, no shapes needed
        return []
    
    def _overlaps_with_box(self, box_min, box_max):
        """Check if this node's region overlaps with the given box."""
        # If any dimension doesn't overlap, the boxes don't overlap
        return not (
            np.any(self.max_corner < box_min) or 
            np.any(self.min_corner > box_max)
        )

class Octree:
    """An octree for efficient spatial partitioning of a point cloud."""
    
    def __init__(self, points, max_points_per_node=10, min_node_size=0.05):
        """
        Initialize an octree with the given points.
        
        Args:
            points: Array of 3D points (N,3)
            max_points_per_node: Maximum number of points in a node before splitting
            min_node_size: Minimum size of a node before stopping subdivision
        """
        # Calculate bounding box of all points
        min_corner = np.min(points, axis=0)
        max_corner = np.max(points, axis=0)
        
        # Calculate center and size of root node
        center = (min_corner + max_corner) / 2
        size = np.max(max_corner - min_corner) * 1.01  # Slightly larger to ensure all points fit
        
        # Create root node
        self.root = OctreeNode(center, size, max_points_per_node, min_node_size)
        
        # Insert all points into the octree
        for i, point in enumerate(points):
            self.root.insert(i, point)
    
    def get_collision_shapes(self, robot_min, robot_max):
        """
        Get collision shapes for regions of the octree that overlap with the robot.
        
        Args:
            robot_min: Minimum corner of the robot's bounding box
            robot_max: Maximum corner of the robot's bounding box
            
        Returns:
            List of (center, size, points) tuples for collision shapes
        """
        return self.root.get_collision_shapes(robot_min, robot_max)

def create_point_cloud_cube(center, size, num_points_per_side):
    """
    Create a point cloud cube centered at a specific location.
    
    Args:
        center: 3D center position of the cube
        size: Size of the cube (same in all dimensions)
        num_points_per_side: Number of points per side
    
    Returns:
        A list of points forming a cube
    """
    total_points = num_points_per_side**3
    points = np.zeros((total_points, 3))  # Shape: (N, 3)
    
    # Calculate the step size for each dimension
    step = size / (num_points_per_side - 1) if num_points_per_side > 1 else 0
    
    # Generate points in a cube pattern
    idx = 0
    for i in range(num_points_per_side):
        x = center[0] - size/2 + i * step
        for j in range(num_points_per_side):
            y = center[1] - size/2 + j * step
            for k in range(num_points_per_side):
                z = center[2] - size/2 + k * step
                points[idx] = [x, y, z]
                idx += 1
    
    return points


def check_collisions_octree(urdf_path, point_cloud_points, visualize=False, max_points_per_node=20, min_node_size=0.1):
    """
    Check if the robot defined by the URDF collides with points using octree-based spatial partitioning.
    
    Args:
        urdf_path: Path to the URDF file
        point_cloud_points: Array of 3D points (N,3)
        visualize: Whether to visualize the results
        max_points_per_node: Maximum points in an octree node before splitting
        min_node_size: Minimum size of octree nodes
    
    Returns:
        Boolean indicating if there is a collision and the meshcat instance
    """
    func_start_time = time.time()
    
    # Start Meshcat server only if visualization is enabled
    t0 = time.time()
    mc_vis = None
    if visualize:
        mc_vis = StartMeshcat()
        mc_vis.Delete()  # Clear any existing visualization
        print(f"\nMeshcat visualization URL: {mc_vis.web_url()}\n")
        print(f"Starting Meshcat took {(time.time() - t0)*1000:.2f} ms")
    
    # Create a diagram for collision checking (and visualization if needed)
    t1 = time.time()
    builder = DiagramBuilder()
    # Use appropriate time step based on whether visualization is needed
    time_step = 0.001 if visualize else 0.0 
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    print(f"Creating MultibodyPlant took {(time.time() - t1)*1000:.2f} ms")
    
    # Load the URDF
    t2 = time.time()
    print(f"Loading URDF from: {urdf_path}")
    parser = Parser(plant)
    
    try:
        model = parser.AddModelsFromUrl(f"file://{urdf_path}")
        print(f"Successfully loaded URDF model with ID: {model[0]}")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        try:
            model = parser.AddModelFromFile(urdf_path)
            print(f"Successfully loaded URDF model with direct file loading, ID: {model}")
        except Exception as e2:
            print(f"Still could not load URDF: {e2}")
            return False, None
    
    print(f"Loading URDF took {(time.time() - t2)*1000:.2f} ms")
    
    # Add visualizer if needed
    meshcat_vis = None
    if visualize:
        meshcat_vis = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            scene_graph=scene_graph,
            meshcat=mc_vis
        )
    
    # Build octree from point cloud
    t3 = time.time()
    octree = Octree(point_cloud_points, max_points_per_node, min_node_size)
    print(f"Building octree took {(time.time() - t3)*1000:.2f} ms")
    
    # Create a separate scene to calculate the robot's bounding box accurately
    # This way we can finalize the plant and get proper joint positions without issues
    print("Creating separate scene to calculate robot bounding box...")
    bbox_calc_time = time.time()
    
    # Import BodyIndex in local scope to avoid any issues with the global import
    from pydrake.multibody.tree import BodyIndex as LocalBodyIndex
    
    # Create a separate builder, plant, and scene graph just for bounding box calculation
    bbox_builder = DiagramBuilder()
    bbox_plant, bbox_scene_graph = AddMultibodyPlantSceneGraph(bbox_builder, time_step=0.0)
    
    # Load the same URDF
    bbox_parser = Parser(bbox_plant)
    try:
        bbox_model = bbox_parser.AddModelsFromUrl(f"file://{urdf_path}")
        print(f"Loaded robot model for bounding box calculation")
    except Exception as e:
        print(f"Error loading URDF for bounding box calculation: {e}")
        # Fall back to point cloud-based estimation if we can't load the robot
        point_cloud_min = np.min(point_cloud_points, axis=0)
        point_cloud_max = np.max(point_cloud_points, axis=0)
        point_cloud_center = (point_cloud_min + point_cloud_max) / 2
        robot_min = point_cloud_center - np.array([1.0, 1.0, 1.0])
        robot_max = point_cloud_center + np.array([1.0, 1.0, 1.0])
        print(f"Using fallback bounding box from point cloud: {robot_min} to {robot_max}")
        
        # Skip the rest of the bounding box calculation
    else:
        # We successfully loaded the robot, so finalize the plant
        bbox_plant.Finalize()
        
        # Now build the diagram and create a simulator
        bbox_diagram = bbox_builder.Build()
        bbox_simulator = Simulator(bbox_diagram)
        bbox_context = bbox_simulator.get_mutable_context()
        bbox_plant_context = bbox_plant.GetMyContextFromRoot(bbox_context)
        
        # Set the same joint positions we'll use for collision checking
        if bbox_plant.num_positions() > 0:
            positions = np.zeros(bbox_plant.num_positions())
            if bbox_plant.num_positions() > 2:  # Make sure we have enough positions
                positions[2] = -0.4
            if bbox_plant.num_positions() > 4:  # Make sure we have enough positions
                positions[4] = 0.4
            
            try:
                bbox_plant.SetPositions(bbox_plant_context, positions)
                print(f"Set {bbox_plant.num_positions()} joint positions for bounding box calculation")
            except Exception as e:
                print(f"Warning: Could not set positions: {e}")
        
        # Calculate bounding box by examining all bodies in the plant
        min_corner = np.array([float('inf'), float('inf'), float('inf')])
        max_corner = np.array([float('-inf'), float('-inf'), float('-inf')])
        
        # Add safety margin to ensure we don't miss any collisions
        safety_margin = 0.1
        body_positions = []
        
        # Iterate through all bodies to find their positions
        body_count = bbox_plant.num_bodies()
        for i in range(body_count):
            try:
                # Create a proper BodyIndex object using the locally imported class
                body = bbox_plant.get_body(LocalBodyIndex(i))
                
                # Skip the world body
                if body.index() == bbox_plant.world_body().index():
                    continue
                
                # Get the pose of this body
                body_pose = bbox_plant.EvalBodyPoseInWorld(bbox_plant_context, body)
                position = body_pose.translation()
                body_positions.append(position)
                
                # Update bounding box
                min_corner = np.minimum(min_corner, position)
                max_corner = np.maximum(max_corner, position)
                
            except Exception as e:
                print(f"Warning: Could not process body {i}: {e}")
        
        if body_positions:
            # We found at least one body, calculate the encompassing box
            # Expand by safety margin
            robot_min = min_corner - safety_margin
            robot_max = max_corner + safety_margin
            print(f"Calculated robot bounding box from {len(body_positions)} bodies: {robot_min} to {robot_max}")
        else:
            # No bodies found, use fallback
            point_cloud_min = np.min(point_cloud_points, axis=0)
            point_cloud_max = np.max(point_cloud_points, axis=0)
            point_cloud_center = (point_cloud_min + point_cloud_max) / 2
            robot_min = point_cloud_center - np.array([1.0, 1.0, 1.0])
            robot_max = point_cloud_center + np.array([1.0, 1.0, 1.0])
            print(f"No bodies found, using fallback bounding box: {robot_min} to {robot_max}")
    
    print(f"Bounding box calculation took {(time.time() - bbox_calc_time)*1000:.2f} ms")
    
    # Get collision shapes from octree that overlap with the robot
    t4 = time.time()
    collision_shapes = octree.get_collision_shapes(robot_min, robot_max)
    print(f"Found {len(collision_shapes)} potential collision regions")
    print(f"Getting collision shapes took {(time.time() - t4)*1000:.2f} ms")
    
    # Register collision geometries for the filtered regions
    t5 = time.time()
    point_to_geometry_id = {}
    region_to_points = {}
    
    # Count total points in collision regions
    total_points_in_regions = sum(len(points) for _, _, points in collision_shapes)
    print(f"Points in collision regions: {total_points_in_regions} out of {len(point_cloud_points)}")
    print(f"Octree reduced collision checks by {100 * (1 - total_points_in_regions / len(point_cloud_points)):.1f}%")
    
    # Actually create the collision geometries
    all_region_ids = []
    for i, (center, size, points) in enumerate(collision_shapes):
        if size < 0.02:  # Very small nodes - use sphere for individual points
            # Create single geometry for all points in this small region
            sphere = Sphere(0.01)  # Slightly larger radius for these concentrated regions
            instance_name = f"octree_region_{i}"
            friction = CoulombFriction(0.9, 0.8)
            
            # Register as collision geometry
            region_id = plant.RegisterCollisionGeometry(
                plant.world_body(),
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
            region_id = plant.RegisterCollisionGeometry(
                plant.world_body(),
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
    print(f"Adding collision geometries took {(time.time() - t5)*1000:.2f} ms")
    
    # Now finalize the plant
    t6 = time.time()
    plant.Finalize()
    print(f"Finalizing plant took {(time.time() - t6)*1000:.2f} ms")
    
    # Build the diagram
    t7 = time.time()
    diagram = builder.Build()
    print(f"Building diagram took {(time.time() - t7)*1000:.2f} ms")
    
    # Create the simulator
    t8 = time.time()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)
    
    # Set joint positions
    if plant.num_positions() > 0:
        positions = np.zeros(plant.num_positions())
        if plant.num_positions() > 2:
            positions[2] = -0.4
        if plant.num_positions() > 4:
            positions[4] = 0.4
            
        try:
            plant.SetPositions(plant_context, positions)
            print(f"Set joint positions for collision checking")
        except RuntimeError as e:
            print(f"Warning: Could not set joint positions: {e}")
    
    print(f"Creating simulator took {(time.time() - t8)*1000:.2f} ms")
    
    # Initialize visualization if enabled
    if visualize:
        try:
            simulator.Initialize()
            print("Successfully initialized visualization")
        except RuntimeError as e:
            print(f"Warning: Simulator initialization error: {e}")
            try:
                meshcat_vis.PublishCallback(context)
                print("Published robot visualization via callback")
            except Exception as e2:
                print(f"Visualization publishing error: {e2}")
    
    # Get the query object for collision detection
    query_object = scene_graph.get_query_output_port().Eval(
        scene_graph.GetMyContextFromRoot(context))
    
    # Start timing collision detection
    collision_start_time = time.time()
    
    # Check for collisions
    collision_pairs = query_object.ComputePointPairPenetration()
    
    # Identify which regions and points are potentially colliding (broad phase)
    colliding_region_ids = set()
    for pair in collision_pairs:
        colliding_region_ids.add(pair.id_A)
        colliding_region_ids.add(pair.id_B)
    
    # Get the points that are in potentially colliding regions
    potentially_colliding_points = set()
    points_to_check = []
    for region_id in colliding_region_ids:
        if region_id in region_to_points:
            # Add all points from potentially colliding regions
            point_indices = region_to_points[region_id]
            potentially_colliding_points.update(point_indices)
            # Track the actual coordinates of these points for the narrow phase
            for idx in point_indices:
                points_to_check.append((idx, point_cloud_points[idx]))
    
    # End timing broad phase collision detection
    broad_phase_end_time = time.time()
    broad_phase_time_ms = (broad_phase_end_time - collision_start_time) * 1000.0
    print(f"\nBroad phase collision detection took {broad_phase_time_ms:.2f} ms")
    print(f"Found {len(collision_pairs)} collision pairs")
    print(f"Number of potentially colliding regions: {len(colliding_region_ids)}")
    print(f"Number of potentially colliding points: {len(potentially_colliding_points)}")
    
    # ---- NARROW PHASE: Check individual points precisely ----
    # Now perform a more precise check on the potentially colliding points
    narrow_phase_start_time = time.time()
    
    if len(potentially_colliding_points) > 0:
        print("\nPerforming narrow-phase collision detection on candidate points...")
        
        # Create a fresh plant and scene graph for the narrow phase check
        narrow_builder = DiagramBuilder()
        narrow_plant, narrow_scene_graph = AddMultibodyPlantSceneGraph(narrow_builder, time_step=time_step)
        
        # Load the same URDF using the same method as the broad phase
        narrow_parser = Parser(narrow_plant)
        try:
            # First try AddModelsFromUrl like we did in the broad phase
            narrow_model = narrow_parser.AddModelsFromUrl(f"file://{urdf_path}")
            print(f"Loaded robot model for narrow phase: {narrow_model}")
        except Exception as e:
            print(f"Error loading URDF for narrow phase: {e}")
            try:
                # Fall back to other methods
                narrow_model = narrow_parser.AddModelFromFile(urdf_path)
                print(f"Loaded robot model (fallback) for narrow phase: {narrow_model}")
            except Exception as e2:
                print(f"Still could not load URDF for narrow phase: {e2}")
                return False, None
        
        # Create individual collision geometries only for potentially colliding points
        narrow_sphere_radius = 0.005  # 5mm radius, same as naive approach
        narrow_point_to_id = {}
        
        for i, (point_idx, point_coords) in enumerate(points_to_check):
            sphere = Sphere(narrow_sphere_radius)
            instance_name = f"narrow_point_{i}"
            friction = CoulombFriction(0.9, 0.8)
            
            sphere_id = narrow_plant.RegisterCollisionGeometry(
                narrow_plant.world_body(),
                RigidTransform(point_coords),
                sphere,
                instance_name,
                friction
            )
            
            narrow_point_to_id[point_idx] = sphere_id
        
        # Finalize the narrow phase plant
        narrow_plant.Finalize()
        
        # Build the diagram and create a simulator
        narrow_diagram = narrow_builder.Build()
        narrow_simulator = Simulator(narrow_diagram)
        narrow_context = narrow_simulator.get_mutable_context()
        narrow_plant_context = narrow_plant.GetMyContextFromRoot(narrow_context)
        
        # Set the same joint positions
        if narrow_plant.num_positions() > 0:
            narrow_plant.SetPositions(narrow_plant_context, positions)
        
        # Get the narrow phase query object
        narrow_query_object = narrow_scene_graph.get_query_output_port().Eval(
            narrow_scene_graph.GetMyContextFromRoot(narrow_context))
        
        # Create a mapping from geometry ID to point index for quick lookup
        point_id_to_idx = {sphere_id: point_idx for point_idx, sphere_id in narrow_point_to_id.items()}
        
        # Check for collisions in the narrow phase
        print("Running narrow phase collision detection on individual points...")
        narrow_collision_pairs = narrow_query_object.ComputePointPairPenetration()
        print(f"Found {len(narrow_collision_pairs)} collision pairs in narrow phase")
        
        # Much simpler approach: Just look for any collision pair that involves one of our point IDs
        # We don't need to know which one is the robot - if it's not our point, it must be the robot
        colliding_points = set()
        
        for pair in narrow_collision_pairs:
            geom_A = pair.id_A
            geom_B = pair.id_B
            penetration = pair.depth
            
            # Check if either geometry is one of our points
            if geom_A in point_id_to_idx:
                point_idx = point_id_to_idx[geom_A]
                colliding_points.add(point_idx)
                print(f"  Point {point_idx} collides with penetration depth {penetration:.6f}")
            
            if geom_B in point_id_to_idx:
                point_idx = point_id_to_idx[geom_B]
                colliding_points.add(point_idx)
                print(f"  Point {point_idx} collides with penetration depth {penetration:.6f}")
        
        # Print collision summary
        print(f"Found {len(colliding_points)} colliding points")
        
        narrow_phase_end_time = time.time()
        narrow_phase_time_ms = (narrow_phase_end_time - narrow_phase_start_time) * 1000.0
        
        print(f"Narrow phase collision detection took {narrow_phase_time_ms:.2f} ms")
        print(f"Number of actually colliding points (after narrow phase): {len(colliding_points)}")
        print(f"Narrow phase eliminated {len(potentially_colliding_points) - len(colliding_points)} false positives")
    else:
        # No potential collisions in broad phase
        colliding_points = set()
        print("No potential collisions found in broad phase")
    
    # End timing for total collision detection
    collision_end_time = time.time()
    total_collision_time_ms = (collision_end_time - collision_start_time) * 1000.0
    print(f"Total collision detection time: {total_collision_time_ms:.2f} ms")
    
    # Calculate total function time
    func_end_time = time.time()
    print(f"Total collision checking function time: {(func_end_time - func_start_time):.2f} seconds")
    
    # Add point cloud visualization if enabled
    if visualize:
        print(f"\nRobot visualization should now be visible in Meshcat at: {mc_vis.web_url()}")
        print("Adding point cloud visualization...")
        
        # Show only the final collision results, not the intermediate regions
        # We want to focus on the precise results after the narrow phase
        
        # Add a visualization of the robot's bounding box used for broad phase filtering
        bbox_min = robot_min
        bbox_max = robot_max
        bbox_center = (bbox_min + bbox_max) / 2
        bbox_size = bbox_max - bbox_min
        
        # Create a semi-transparent box to show the bounding volume used for filtering
        mc_vis.SetObject("robot_bounding_box", 
                          Box(bbox_size[0], bbox_size[1], bbox_size[2]), 
                          Rgba(0.2, 0.6, 0.9, 0.1))  # Blue, very transparent
        mc_vis.SetTransform("robot_bounding_box", RigidTransform(bbox_center))
        
        # Add all points in the potentially colliding regions for reference
        sphere_radius = 0.005  # 5mm radius
        
        # First add non-colliding points as very transparent blue
        # for i in range(len(point_cloud_points)):
        #     if i not in potentially_colliding_points:
        #         point_name = f"point_cloud/outside_regions/{i}"
        #         color = Rgba(0.2, 0.2, 0.8, 0.05)  # Blue, very transparent
        #         mc_vis.SetObject(point_name, Sphere(sphere_radius*0.8), color)
        #         mc_vis.SetTransform(point_name, RigidTransform(point_cloud_points[i]))
        
        # Add points that were in colliding regions but were filtered out by narrow phase
        for i in potentially_colliding_points:
            if i not in colliding_points:
                point_name = f"point_cloud/filtered/{i}"
                color = Rgba(0.8, 0.8, 0.2, 0.3)  # Yellow, semi-transparent
                mc_vis.SetObject(point_name, Sphere(sphere_radius), color)
                mc_vis.SetTransform(point_name, RigidTransform(point_cloud_points[i]))
        
        # Finally, add the actually colliding points as red and more prominent
        for i in colliding_points:
            point_name = f"point_cloud/colliding/{i}"
            color = Rgba(0.9, 0.1, 0.1, 0.9)  # Red, opaque
            mc_vis.SetObject(point_name, Sphere(sphere_radius*2.0), color)
            mc_vis.SetTransform(point_name, RigidTransform(point_cloud_points[i]))
    
    return len(colliding_points) > 0, mc_vis if visualize else None


def check_collisions_naive(urdf_path, point_cloud_points, visualize=False):
    """
    Check if the robot defined by the URDF collides with points using a naive approach.
    Creates one collision geometry per point.
    
    Args:
        urdf_path: Path to the URDF file
        point_cloud_points: Array of 3D points (N,3)
        visualize: Whether to visualize the results
    
    Returns:
        Boolean indicating if there is a collision and the meshcat instance
    """
    func_start_time = time.time()
    
    # Start Meshcat server only if visualization is enabled
    t0 = time.time()
    mc_vis = None
    if visualize:
        mc_vis = StartMeshcat()
        mc_vis.Delete()  # Clear any existing visualization
        print(f"\nMeshcat visualization URL: {mc_vis.web_url()}\n")
        print(f"Starting Meshcat took {(time.time() - t0)*1000:.2f} ms")
    
    # Create a diagram for collision checking
    t1 = time.time()
    builder = DiagramBuilder()
    time_step = 0.001 if visualize else 0.0 
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    print(f"Creating MultibodyPlant took {(time.time() - t1)*1000:.2f} ms")
    
    # Load the URDF
    t2 = time.time()
    print(f"Loading URDF from: {urdf_path}")
    parser = Parser(plant)
    
    try:
        model = parser.AddModelsFromUrl(f"file://{urdf_path}")
        print(f"Successfully loaded URDF model with ID: {model[0]}")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        try:
            model = parser.AddModelFromFile(urdf_path)
            print(f"Successfully loaded URDF model with direct file loading, ID: {model}")
        except Exception as e2:
            print(f"Still could not load URDF: {e2}")
            return False, None
    
    print(f"Loading URDF took {(time.time() - t2)*1000:.2f} ms")
    
    # Add visualizer if needed
    meshcat_vis = None
    if visualize:
        meshcat_vis = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            scene_graph=scene_graph,
            meshcat=mc_vis
        )
    
    # Register a collision geometry for each point
    t4 = time.time()
    sphere_radius = 0.005  # 5mm radius
    point_to_geometry_id = {}
    
    for i, point in enumerate(point_cloud_points):
        sphere = Sphere(sphere_radius)
        instance_name = f"point_sphere_{i}"
        friction = CoulombFriction(0.9, 0.8)
        
        sphere_id = plant.RegisterCollisionGeometry(
            plant.world_body(),
            RigidTransform(point),
            sphere,
            instance_name,
            friction
        )
        
        point_to_geometry_id[i] = (sphere_id, point)
    
    print(f"Adding {len(point_cloud_points)} collision spheres took {(time.time() - t4)*1000:.2f} ms")
    
    # Finalize the plant
    t5 = time.time()
    plant.Finalize()
    print(f"Finalizing plant took {(time.time() - t5)*1000:.2f} ms")
    
    # Build the diagram
    t6 = time.time()
    diagram = builder.Build()
    print(f"Building diagram took {(time.time() - t6)*1000:.2f} ms")
    
    # Create simulator
    t7 = time.time()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)
    
    # Set joint positions
    if plant.num_positions() > 0:
        positions = np.zeros(plant.num_positions())
        if plant.num_positions() > 2:
            positions[2] = -0.4
        if plant.num_positions() > 4:
            positions[4] = 0.4
            
        try:
            plant.SetPositions(plant_context, positions)
            print(f"Set joint positions for collision checking")
        except RuntimeError as e:
            print(f"Warning: Could not set joint positions: {e}")
    
    print(f"Creating simulator took {(time.time() - t7)*1000:.2f} ms")
    
    # Initialize visualization if enabled
    if visualize:
        try:
            simulator.Initialize()
            print("Successfully initialized visualization")
        except RuntimeError as e:
            print(f"Warning: Simulator initialization error: {e}")
            try:
                meshcat_vis.PublishCallback(context)
                print("Published robot visualization via callback")
            except Exception as e2:
                print(f"Visualization publishing error: {e2}")
    
    # Get the query object for collision detection
    query_object = scene_graph.get_query_output_port().Eval(
        scene_graph.GetMyContextFromRoot(context))
    
    # Start timing collision detection
    collision_start_time = time.time()
    
    # Check for collisions
    collision_pairs = query_object.ComputePointPairPenetration()
    
    # Set of colliding geometry IDs
    colliding_ids = set()
    for pair in collision_pairs:
        colliding_ids.add(pair.id_A)
        colliding_ids.add(pair.id_B)
    
    # Identify which points are colliding
    colliding_points = set()
    non_colliding_points = set()
    
    for i, (collision_id, point) in point_to_geometry_id.items():
        if collision_id in colliding_ids:
            colliding_points.add(i)
        else:
            non_colliding_points.add(i)
    
    # End timing and calculate elapsed time
    collision_end_time = time.time()
    collision_time_ms = (collision_end_time - collision_start_time) * 1000.0
    print(f"\nCollision detection took {collision_time_ms:.2f} ms for {len(point_cloud_points)} points")
    
    # Calculate total function time
    func_end_time = time.time()
    print(f"Total collision checking function time: {(func_end_time - func_start_time):.2f} seconds")
    
    # Show statistics
    print(f"Found {len(collision_pairs)} collision pairs")
    print(f"Number of colliding points: {len(colliding_points)}")
    print(f"Number of non-colliding points: {len(non_colliding_points)}")
    
    # Add point cloud visualization if enabled
    if visualize:
        print(f"\nRobot visualization should now be visible in Meshcat at: {mc_vis.web_url()}")
        print("Adding point cloud visualization...")
        
        # Add colliding points as red
        for i in colliding_points:
            point_name = f"point_cloud/colliding/{i}"
            red_color = Rgba(0.9, 0.1, 0.1, 0.9)  # Red, more opaque
            mc_vis.SetObject(point_name, Sphere(sphere_radius*1.5), red_color)  
            mc_vis.SetTransform(point_name, RigidTransform(point_cloud_points[i]))
        
        # Add non-colliding points as blue and more transparent
        for i in non_colliding_points:
            point_name = f"point_cloud/non_colliding/{i}"
            blue_color = Rgba(0.1, 0.1, 0.8, 0.2)  # Blue, transparent
            mc_vis.SetObject(point_name, Sphere(sphere_radius), blue_color)
            mc_vis.SetTransform(point_name, RigidTransform(point_cloud_points[i]))
    
    return len(colliding_points) > 0, mc_vis if visualize else None

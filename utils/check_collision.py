#!/usr/bin/env python3

import numpy as np
import os
import sys
import time
import argparse

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    RigidTransform,
    Simulator,
    GeometryInstance,
    Sphere,
    Role,
    CoulombFriction,
    StartMeshcat,
    MeshcatVisualizer,
    Rgba,
    InverseDynamicsController,
)

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

def check_collisions(urdf_path, point_cloud_points, visualize=False):
    """
    Check if the robot defined by the URDF collides with points and visualize.
    
    Args:
        urdf_path: Path to the URDF file
        point_cloud_points: Array of 3D points (N,3)
    
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
    if visualize:
        print(f"Starting Meshcat took {(time.time() - t0)*1000:.2f} ms")
    
    # Create a diagram for collision checking (and visualization if enabled)
    t1 = time.time()
    builder = DiagramBuilder()
    # Use appropriate time step based on whether visualization is needed
    time_step = 0.001 if visualize else 0.0 
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    print(f"Creating MultibodyPlant took {(time.time() - t1)*1000:.2f} ms")
    
    # Give better error messages for missing meshes
    t2 = time.time()
    print(f"Loading URDF from: {urdf_path}")
    parser = Parser(plant)
    
    # Load the URDF - retry with different attempts if necessary
    try:
        # Normal URL loading
        model = parser.AddModelsFromUrl(f"file://{urdf_path}")
        print(f"Successfully loaded URDF model with ID: {model[0]}")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        try:
            # Direct file loading
            model = parser.AddModelFromFile(urdf_path)
            print(f"Successfully loaded URDF model with direct file loading, ID: {model}")
        except Exception as e2:
            print(f"Still could not load URDF: {e2}")
            print("Will attempt to proceed anyway")
    print(f"Loading URDF took {(time.time() - t2)*1000:.2f} ms")
    
    # Add collision spheres for each point in the point cloud
    t3 = time.time()
    sphere_radius = 0.005  # 5mm radius
    
    # Keep track of sphere geometries and their points for later collision checking
    sphere_geometries = []
    point_to_geometry_id = {}
    
    for i, point in enumerate(point_cloud_points):
        # Create a small sphere at each point
        sphere = Sphere(sphere_radius)
        
        # Create an instance for the sphere
        instance_name = f"point_sphere_{i}"
        
        # Register the geometry with the scene graph for collision detection
        friction = CoulombFriction(0.9, 0.8)  # static and dynamic friction
        sphere_id = plant.RegisterCollisionGeometry(
            plant.world_body(),
            RigidTransform(point),
            sphere,
            instance_name,
            friction
        )
        
        sphere_geometries.append(sphere_id)
        point_to_geometry_id[i] = (sphere_id, point)
    
    print(f"Adding {len(point_cloud_points)} collision spheres took {(time.time() - t3)*1000:.2f} ms")
    
    # Finalize the plant
    t4 = time.time()
    plant.Finalize()
    print(f"Finalizing plant took {(time.time() - t4)*1000:.2f} ms")
    
    # Add a visualizer only if visualization is enabled
    t5 = time.time()
    meshcat_vis = None
    if visualize:
        meshcat_vis = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            scene_graph=scene_graph,
            meshcat=mc_vis
        )
        print(f"Adding visualizer took {(time.time() - t5)*1000:.2f} ms")
    
    # Because of the mass matrix errors, we need to publish to Meshcat directly
    # without trying to simulate dynamics; the meshcat_vis will handle visualization 
    # as long as we can initialize it properly
    
    # Build the diagram
    t6 = time.time()
    diagram = builder.Build()
    print(f"Building diagram took {(time.time() - t6)*1000:.2f} ms")
    
    # Create a simulator with fixed-step integration to avoid some numerical issues
    t7 = time.time()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    print(f"Creating simulator took {(time.time() - t7)*1000:.2f} ms")
    
    # Get the MultibodyPlant context
    plant_context = plant.GetMyContextFromRoot(context)
    
    # Set all joint positions to 0 if possible
    t8 = time.time()
    positions = np.zeros(plant.num_positions())
    positions[2] = -0.4
    positions[4] = 0.4
    if plant.num_positions() > 0:
        try:
            plant.SetPositions(plant_context, positions)
            print(f"Set all {plant.num_positions()} joint positions to custom values")
        except RuntimeError as e:
            print(f"Warning: Could not set joint positions: {e}")
    print(f"Setting joint positions took {(time.time() - t8)*1000:.2f} ms")
    
    # Initialize visualization only if enabled
    if visualize:
        try:
            # Initialize the simulator to publish visualization
            simulator.Initialize()
            print("Successfully initialized visualization. Robot should be visible.")
        except RuntimeError as e:
            print(f"Warning: Simulator initialization error: {e}")
            print("We'll try a different approach to visualize the robot...")
            
            # Let's try direct publishing to the visualizer
            try:
                # Force MeshcatVisualizer to publish current state
                meshcat_vis.PublishCallback(context)
                print("Published robot visualization via callback.")
            except Exception as e2:
                print(f"Visualization publishing error: {e2}")
    
    # Even if visualization fails with simulation, we can still perform collision checking
    
    # Get the SceneGraph's query object for collision detection
    query_object = scene_graph.get_query_output_port().Eval(
        scene_graph.GetMyContextFromRoot(context))
    
    # Start timing the collision detection
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
    
    # Add point cloud visualization only if visualization is enabled
    if visualize:
        print(f"\nRobot visualization should now be visible in Meshcat at: {mc_vis.web_url()}")
        print("Adding point cloud visualization...")
        
        # Add non-colliding points as blue spheres
        for i in non_colliding_points:
            _, point = point_to_geometry_id[i]
            point_name = f"point_cloud/non_colliding/{i}"
            blue_color = Rgba(0.2, 0.2, 0.8, 0.7)  # Blue, semi-transparent
            mc_vis.SetObject(point_name, Sphere(sphere_radius), blue_color)
            mc_vis.SetTransform(point_name, RigidTransform(point))
        
        # Add colliding points as red and slightly larger
        for i in colliding_points:
            _, point = point_to_geometry_id[i]
            point_name = f"point_cloud/colliding/{i}"
            red_color = Rgba(0.9, 0.1, 0.1, 0.9)  # Red, more opaque
            mc_vis.SetObject(point_name, Sphere(sphere_radius*1.5), red_color)  
            mc_vis.SetTransform(point_name, RigidTransform(point))
    
    print(f"Found {len(collision_pairs)} collision pairs")
    print(f"Number of colliding points: {len(colliding_points)}")
    print(f"Number of non-colliding points: {len(non_colliding_points)}")
    
    # Show more details about the first few collision pairs
    if len(collision_pairs) > 0:
        print("\nSample collisions:")
        for i, pair in enumerate(collision_pairs[:10]):  # Show only first 10 for brevity
            print(f"Collision {i+1}: IDs {pair.id_A} and {pair.id_B}, depth: {pair.depth:.6f}")
        if len(collision_pairs) > 10:
            print(f"... and {len(collision_pairs) - 10} more collision pairs")
        return True, mc_vis if visualize else None
    
    return False, mc_vis if visualize else None

def main():
    main_start_time = time.time()
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Check collisions between a robot and a point cloud')
    parser.add_argument('--visualize', action='store_true', help='Enable visualization')
    parser.add_argument('--points', type=int, default=20, help='Number of points per side of the cube')
    args = parser.parse_args()
    
    # Get the path to the URDF file
    args_time = time.time()
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../assets/devkit_base_descr.urdf"))
    
    # Ensure the URDF file exists
    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        return 1
    print(f"Parsing args took {(time.time() - args_time)*1000:.2f} ms")
        
    # Track total execution time as well
    total_start_time = time.time()
    
    print(f"Using URDF: {urdf_path}")
    
    # Create a point cloud cube
    cube_start_time = time.time()
    center = [-0.4, 0, 1.7]  # Center position
    size = 1.0
    num_points_per_side = args.points  # Use command line argument for points
    
    print(f"Creating point cloud cube with {num_points_per_side}^3 = {num_points_per_side**3} points")
    print(f"Cube center: {center}, size: {size}")
    
    points = create_point_cloud_cube(center, size, num_points_per_side)
    print(f"Creating point cloud took {(time.time() - cube_start_time)*1000:.2f} ms")
    
    # Check for collisions (with or without visualization)
    if args.visualize:
        print("Checking for collisions and visualizing...")
    else:
        print("Checking for collisions (no visualization)...")
    
    has_collision, meshcat = check_collisions(urdf_path, points, visualize=args.visualize)
    
    if has_collision:
        print("\nRobot collides with the point cloud!")
        print("Red points indicate collision with the robot.")
        if args.visualize:
            print(f"\nVisualization URL: {meshcat.web_url()}")
            print("Please open this URL in your browser if it didn't open automatically.")
    else:
        print("\nNo collision detected between robot and point cloud.")
    
    # Calculate and display total execution time
    total_end_time = time.time()
    total_time_sec = total_end_time - total_start_time
    main_time_sec = total_end_time - main_start_time
    print(f"\nTotal script execution time: {total_time_sec:.2f} seconds")
    print(f"Main function execution time: {main_time_sec:.2f} seconds")
    
    # Keep the script running to maintain visualization only if visualizing
    if args.visualize and meshcat is not None:
        input("\nPress Enter to exit...")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

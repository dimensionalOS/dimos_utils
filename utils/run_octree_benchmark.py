#!/usr/bin/env python3

import numpy as np
import os
import sys
import time
import argparse

from octree_collision import (
    create_point_cloud_cube,
    check_collisions_octree,
    check_collisions_naive
)

def main():
    main_start_time = time.time()
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Benchmark collision detection with and without octree optimization')
    parser.add_argument('--visualize', action='store_true', help='Enable visualization')
    parser.add_argument('--points', type=int, default=20, help='Number of points per side of the cube')
    parser.add_argument('--max-points-per-node', type=int, default=10, help='Maximum points in an octree node before splitting')
    parser.add_argument('--min-node-size', type=float, default=0.01, help='Minimum size of octree nodes')
    parser.add_argument('--naive', action='store_true', help='Only run the naive implementation')
    parser.add_argument('--octree', action='store_true', help='Only run the octree implementation')
    args = parser.parse_args()
    
    # Get the path to the URDF file
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../assets/devkit_base_descr.urdf"))
    
    # Ensure the URDF file exists
    if not os.path.exists(urdf_path):
        print(f"URDF file not found at: {urdf_path}")
        return 1
    
    print(f"Using URDF: {urdf_path}")
    
    # Create a point cloud cube
    center = [-0.0, 0, 1.5]  # Center position
    size = 1.5  # 220cm cube
    num_points_per_side = args.points
    
    print(f"Creating point cloud cube with {num_points_per_side}^3 = {num_points_per_side**3} points")
    print(f"Cube center: {center}, size: {size}")
    
    create_time = time.time()
    points = create_point_cloud_cube(center, size, num_points_per_side)
    print(f"Creating point cloud took {(time.time() - create_time)*1000:.2f} ms")
    
    # Check if we should run one or both implementations
    run_naive = args.naive or (not args.octree and not args.naive)
    run_octree = args.octree or (not args.octree and not args.naive)
    
    # Track benchmark results
    naive_time = None
    has_collision_naive = None
    octree_time = None
    has_collision_octree = None
    meshcat = None
    
    # Run naive implementation if requested
    if run_naive:
        print("\n==== Running naive implementation (one geometry per point) ====")
        benchmark_start = time.time()
        has_collision_naive, meshcat_naive = check_collisions_naive(
            urdf_path, 
            points, 
            visualize=args.visualize
        )
        naive_time = time.time() - benchmark_start
        print(f"\nTotal time with naive approach: {naive_time:.2f} seconds")
        print(f"Collision result: {'Collision detected' if has_collision_naive else 'No collision'}")
        
        if args.visualize:
            meshcat = meshcat_naive
    
    # Run octree implementation if requested
    if run_octree:
        print("\n==== Running octree implementation ====")
        print(f"Parameters: max {args.max_points_per_node} points per node, min size {args.min_node_size}m")
        benchmark_start = time.time()
        has_collision_octree, meshcat_octree = check_collisions_octree(
            urdf_path, 
            points, 
            visualize=args.visualize,
            max_points_per_node=args.max_points_per_node,
            min_node_size=args.min_node_size
        )
        octree_time = time.time() - benchmark_start
        print(f"\nTotal time with octree optimization: {octree_time:.2f} seconds")
        print(f"Collision result: {'Collision detected' if has_collision_octree else 'No collision'}")
        
        if args.visualize:
            meshcat = meshcat_octree
    
    # Calculate improvement if both implementations were run
    if run_naive and run_octree:
        improvement_pct = (naive_time - octree_time) / naive_time * 100
        print(f"\nOctree optimization {('improved' if improvement_pct > 0 else 'worsened')} performance by {abs(improvement_pct):.1f}%")
    
    # Show overall result
    has_collision = has_collision_octree if run_octree else has_collision_naive
    if has_collision:
        print("\nROBOT COLLIDES with the point cloud!")
    else:
        print("\nNo collision detected between robot and point cloud.")
    
    # Calculate total execution time
    total_time = time.time() - main_start_time
    print(f"\nTotal script execution time: {total_time:.2f} seconds")
    
    # Keep the script running to maintain visualization if enabled
    if args.visualize and meshcat is not None:
        input("\nPress Enter to exit...")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

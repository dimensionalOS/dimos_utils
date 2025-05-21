#!/usr/bin/env python3

import numpy as np
import open3d as o3d

# Create a simple point cloud representing a wall/obstacle in front of the robot
def create_wall_point_cloud(output_file="sample_wall.pcd", num_points=1000):
    # Create a grid of points representing a wall
    x = np.random.uniform(-0.6, -0.5, num_points)
    y = np.random.uniform(-0.5, 0.5, num_points)
    z = np.random.uniform(1.3, 1.4, num_points)
    
    # Combine into point cloud
    points = np.column_stack((x, y, z))
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Set random colors for visualization (red-ish)
    colors = np.column_stack([
        np.ones(num_points) * 0.8,  # Red
        np.random.uniform(0, 0.3, num_points),  # Green
        np.random.uniform(0, 0.3, num_points)   # Blue
    ])
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save to file
    o3d.io.write_point_cloud(output_file, pcd)
    print(f"Saved point cloud with {num_points} points to {output_file}")
    
    return output_file

if __name__ == "__main__":
    create_wall_point_cloud()

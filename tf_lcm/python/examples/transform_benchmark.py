#!/usr/bin/env python3
"""
Transform Benchmark Test (Python)

This script benchmarks the performance of transforming 1,000,000 random
3D points from the link6 frame to the world frame using tf_lcm.

It is equivalent to the C++ transform_benchmark.cpp test.
"""

import os
import sys
import time
import signal
import datetime
import numpy as np
import lcm
from dataclasses import dataclass

# Import tf_lcm_py
import tf_lcm_py

# Import LCM message types
import lcm_msgs
from lcm_msgs.tf2_msgs.TFMessage import TFMessage

# Signal handling for clean shutdown
keep_running = True

def signal_handler(signum, frame):
    global keep_running
    print(f"Received signal {signum}, shutting down...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

@dataclass
class Point3D:
    """Simple 3D point structure"""
    x: float
    y: float
    z: float

def generate_random_points(n, min_val=-10.0, max_val=10.0):
    """Generate n random points with coordinates in range [min_val, max_val]"""
    print(f"Generating {n:,} random points...")
    start_time = time.time()
    
    # Using numpy for efficient random generation
    points_array = np.random.uniform(min_val, max_val, (n, 3))
    
    # Convert to list of Point3D for consistent API with C++ version
    points = [Point3D(x=p[0], y=p[1], z=p[2]) for p in points_array]
    
    end_time = time.time()
    print(f"Generated {len(points):,} points in {end_time - start_time:.3f} seconds")
    
    return points

def transform_point(transform, point):
    """
    Transform a point using a transform (quaternion-based transformation)
    Equivalent to the C++ transformPoint function
    """
    # Extract rotation (quaternion)
    qw = transform.transform.rotation.w
    qx = transform.transform.rotation.x
    qy = transform.transform.rotation.y
    qz = transform.transform.rotation.z
    
    # Extract translation
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z
    
    # Extract point coordinates
    px = point.x
    py = point.y
    pz = point.z
    
    # Apply rotation using quaternion-vector multiplication
    # Formula: p' = q * p * q^-1 where p = (0, px, py, pz)
    
    # First compute q * p
    tmp_w = -qx * px - qy * py - qz * pz
    tmp_x = qw * px + qy * pz - qz * py
    tmp_y = qw * py + qz * px - qx * pz
    tmp_z = qw * pz + qx * py - qy * px
    
    # Then compute (q * p) * q^-1 (note: q^-1 = (qw, -qx, -qy, -qz) for unit quaternions)
    rx = tmp_x * qw + tmp_w * (-qx) + tmp_y * (-qz) - tmp_z * (-qy)
    ry = tmp_y * qw + tmp_w * (-qy) + tmp_z * (-qx) - tmp_x * (-qz)
    rz = tmp_z * qw + tmp_w * (-qz) + tmp_x * (-qy) - tmp_y * (-qx)
    
    # Apply translation
    rx += tx
    ry += ty
    rz += tz
    
    return Point3D(x=rx, y=ry, z=rz)

def transform_point_batch(transform, points, use_numpy=True):
    """
    Transform a batch of points using numpy for better performance
    This is an optimized version of the point-by-point transformation
    """
    if not use_numpy:
        # Fall back to point-by-point transformation
        return [transform_point(transform, p) for p in points]
    
    # Extract rotation (quaternion)
    qw = transform.transform.rotation.w
    qx = transform.transform.rotation.x
    qy = transform.transform.rotation.y
    qz = transform.transform.rotation.z
    
    # Extract translation
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z
    
    # Convert points to numpy array if not already
    if isinstance(points[0], Point3D):
        points_array = np.array([[p.x, p.y, p.z] for p in points])
    else:
        points_array = np.array(points)
    
    # Extract point coordinates
    px = points_array[:, 0]
    py = points_array[:, 1]
    pz = points_array[:, 2]
    
    # Apply rotation using quaternion-vector multiplication vectorized
    # First compute q * p
    tmp_w = -qx * px - qy * py - qz * pz
    tmp_x = qw * px + qy * pz - qz * py
    tmp_y = qw * py + qz * px - qx * pz
    tmp_z = qw * pz + qx * py - qy * px
    
    # Then compute (q * p) * q^-1
    rx = tmp_x * qw + tmp_w * (-qx) + tmp_y * (-qz) - tmp_z * (-qy)
    ry = tmp_y * qw + tmp_w * (-qy) + tmp_z * (-qx) - tmp_x * (-qz)
    rz = tmp_z * qw + tmp_w * (-qz) + tmp_x * (-qy) - tmp_y * (-qx)
    
    # Apply translation
    rx += tx
    ry += ty
    rz += tz
    
    # Combine into output array
    result = np.column_stack((rx, ry, rz))
    
    # Convert back to Point3D objects for consistent API
    return [Point3D(x=p[0], y=p[1], z=p[2]) for p in result]

class TransformReceiver:
    def __init__(self):
        # Create a buffer for transforms
        self.buffer = tf_lcm_py.Buffer(30.0)  # 30 seconds cache
        
        # Create LCM instance
        self.lc = lcm.LCM()
        
        # Subscribe to TF channels using the EXACT same channel names as C++
        self.lc.subscribe("tf#tf2_msgs.TFMessage", self.tf_callback)
        self.lc.subscribe("tf_static#tf2_msgs.TFMessage", self.tf_static_callback)
        
        print("TransformReceiver initialized")
        print("Subscribed to 'tf#tf2_msgs.TFMessage' and 'tf_static#tf2_msgs.TFMessage'")
    
    def tf_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {msg.transforms_length} transforms on channel {channel}")
            
            # Process each transform
            for i in range(msg.transforms_length):
                transform = msg.transforms[i]
                print(f"  {transform.header.frame_id} -> {transform.child_frame_id}")
                self.buffer.set_transform(transform, "lcm_publisher", False)
        except Exception as e:
            print(f"Error processing transform: {e}")
    
    def tf_static_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {msg.transforms_length} static transforms on channel {channel}")
            
            # Process each transform
            for i in range(msg.transforms_length):
                transform = msg.transforms[i]
                print(f"  {transform.header.frame_id} -> {transform.child_frame_id}")
                self.buffer.set_transform(transform, "static_publisher", True)
        except Exception as e:
            print(f"Error processing static transform: {e}")

def print_transform(transform):
    """Print transform in a formatted way"""
    print("\nTransform found!")
    print(f"  Header frame: {transform.header.frame_id}")
    print(f"  Child frame: {transform.child_frame_id}")
    print(f"  Timestamp: {transform.header.stamp.sec}.{transform.header.stamp.nsec:09d}")
    print(f"  Translation: ({transform.transform.translation.x:.6f}, "
          f"{transform.transform.translation.y:.6f}, "
          f"{transform.transform.translation.z:.6f})")
    print(f"  Rotation (quaternion): ({transform.transform.rotation.w:.6f}, "
          f"{transform.transform.rotation.x:.6f}, "
          f"{transform.transform.rotation.y:.6f}, "
          f"{transform.transform.rotation.z:.6f})")

def main():
    # Parse command line arguments
    num_points = 1000000  # Default 1 million points
    use_numpy = True     # Use numpy optimization by default
    
    if len(sys.argv) > 1:
        try:
            num_points = int(sys.argv[1])
            if num_points <= 0:
                print("Number of points must be positive")
                return 1
        except ValueError:
            print(f"Invalid argument for number of points: {sys.argv[1]}")
            print(f"Usage: {sys.argv[0]} [num_points] [use_numpy_optimization:0|1]")
            return 1
    
    if len(sys.argv) > 2:
        try:
            use_numpy = bool(int(sys.argv[2]))
        except ValueError:
            print(f"Invalid argument for numpy optimization: {sys.argv[2]}")
            print("Should be 0 (disabled) or 1 (enabled)")
            return 1
    
    print("=== Transform Benchmark Test (Python) ===")
    print(f"Will transform {num_points:,} random points from link6 to world frame")
    print(f"Numpy optimization: {'Enabled' if use_numpy else 'Disabled'}")
    
    # Define the source and target frames
    source_frame = "link6"
    target_frame = "world"
    
    # Create the transform receiver
    receiver = TransformReceiver()
    
    print(f"Waiting for transform between '{source_frame}' and '{target_frame}'...")
    
    # Wait for the transform to become available
    found = False
    attempts = 0
    transform = None
    
    while keep_running and not found and attempts < 60:
        try:
            # Handle LCM messages
            receiver.lc.handle_timeout(100)  # 100ms timeout
            
            # Get current time
            now = datetime.datetime.now()
            
            # Try to get the transform
            if receiver.buffer.can_transform(target_frame, source_frame, now):
                transform = receiver.buffer.lookup_transform(target_frame, source_frame, now)
                found = True
                print_transform(transform)
            else:
                attempts += 1
                time.sleep(0.5)
        except Exception as e:
            attempts += 1
            if attempts % 5 == 0:
                print(f"Exception: {e}")
            time.sleep(0.5)
    
    if not found:
        print(f"Could not find transform between '{source_frame}' and '{target_frame}' "
              f"after {attempts} attempts. Exiting.")
        return 1
    
    # Generate random points
    points = generate_random_points(num_points)
    
    # Transform all points
    print("Transforming points...")
    transformed_points = []
    
    # Start timing
    start_transform = time.time()
    
    # Transform all points (either point-by-point or using numpy batch transform)
    if use_numpy:
        transformed_points = transform_point_batch(transform, points, use_numpy=True)
    else:
        # Transform point-by-point (slower)
        for point in points:
            transformed_points.append(transform_point(transform, point))
    
    # End timing
    end_transform = time.time()
    transform_time = end_transform - start_transform
    
    # Calculate performance metrics
    points_per_second = num_points / transform_time
    
    # Print results
    print("\n=== Benchmark Results ===")
    print(f"Total points transformed: {num_points:,}")
    print(f"Total transform time: {transform_time:.6f} seconds")
    print(f"Performance: {points_per_second:.2f} points/second")
    print(f"Method: {'Numpy vectorized' if use_numpy else 'Point-by-point'}")
    
    # Print first and last transformed point as sanity check
    if transformed_points:
        print(f"\nFirst transformed point: ({transformed_points[0].x:.6f}, "
              f"{transformed_points[0].y:.6f}, {transformed_points[0].z:.6f})")
        
        print(f"Last transformed point: ({transformed_points[-1].x:.6f}, "
              f"{transformed_points[-1].y:.6f}, {transformed_points[-1].z:.6f})")
    
    print("\nBenchmark completed successfully!")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

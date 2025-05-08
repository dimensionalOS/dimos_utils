#!/usr/bin/env python3
"""
Example of broadcasting static transforms using the tf_lcm Python bindings
"""

import time
import datetime
import sys
import os
import lcm
import numpy as np

# Add the parent directory to the path so we can import the module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import LCM message types
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header

# Import tf_lcm module (assumes the library has been built and installed)
import tf_lcm_py

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

def main():
    print("TF_LCM Static Broadcaster Example")
    
    # Create a static broadcaster
    static_broadcaster = tf_lcm_py.StaticTransformBroadcaster()
    
    # Get current time
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    
    # Create a list for storing transforms
    static_transforms = []
    
    # 1. Create map to odom static transform
    map_to_odom = TransformStamped()
    map_to_odom.header = Header()
    map_to_odom.header.frame_id = "map"
    map_to_odom.header.stamp.sec = sec
    map_to_odom.header.stamp.nsec = nsec
    map_to_odom.child_frame_id = "odom"
    
    map_to_odom.transform = Transform()
    map_to_odom.transform.translation = Vector3()
    map_to_odom.transform.rotation = Quaternion()
    
    map_to_odom.transform.translation.x = 10.0
    map_to_odom.transform.translation.y = 5.0
    map_to_odom.transform.translation.z = 0.0
    map_to_odom.transform.rotation = quaternion_from_euler(0, 0, 0.1)  # Slight rotation
    
    static_transforms.append(map_to_odom)
    
    # 2. Create base_link to camera static transform
    base_to_camera = TransformStamped()
    base_to_camera.header = Header()
    base_to_camera.header.frame_id = "base_link"
    base_to_camera.header.stamp.sec = sec
    base_to_camera.header.stamp.nsec = nsec
    base_to_camera.child_frame_id = "camera"
    
    base_to_camera.transform = Transform()
    base_to_camera.transform.translation = Vector3()
    base_to_camera.transform.rotation = Quaternion()
    
    base_to_camera.transform.translation.x = 0.2
    base_to_camera.transform.translation.y = 0.0
    base_to_camera.transform.translation.z = 0.3
    base_to_camera.transform.rotation = quaternion_from_euler(0, 0.1, 0)  # Slight downward pitch
    
    static_transforms.append(base_to_camera)
    
    # 3. Create base_link to lidar static transform
    base_to_lidar = TransformStamped()
    base_to_lidar.header = Header()
    base_to_lidar.header.frame_id = "base_link"
    base_to_lidar.header.stamp.sec = sec
    base_to_lidar.header.stamp.nsec = nsec
    base_to_lidar.child_frame_id = "lidar"
    
    base_to_lidar.transform = Transform()
    base_to_lidar.transform.translation = Vector3()
    base_to_lidar.transform.rotation = Quaternion()
    
    base_to_lidar.transform.translation.x = 0.0
    base_to_lidar.transform.translation.y = 0.0
    base_to_lidar.transform.translation.z = 0.5
    base_to_lidar.transform.rotation = quaternion_from_euler(0, 0, 0)  # Identity rotation
    
    static_transforms.append(base_to_lidar)
    
    # Broadcast all static transforms at once
    static_broadcaster.send_transforms(static_transforms)
    
    print("Broadcasted the following static transforms:")
    for transform in static_transforms:
        print(f"  {transform.header.frame_id} -> {transform.child_frame_id}")
    
    # Sleep to keep the program running
    print("Static transforms have been published. Keeping the program alive...")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("Static broadcaster stopped")

if __name__ == "__main__":
    main()

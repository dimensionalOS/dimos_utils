#!/usr/bin/env python3
"""
Example of broadcasting transforms using the tf_lcm Python bindings
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
    print("TF_LCM Broadcaster Example")
    
    # Create a broadcaster
    broadcaster = tf_lcm_py.TransformBroadcaster()
    
    print("Broadcasting transforms from world->base_link->arm_link1")
    
    rate = 10  # Hz
    
    try:
        while True:
            # Get current time
            now = time.time()
            sec = int(now)
            nsec = int((now - sec) * 1e9)
            
            # Create transforms
            transforms = []
            
            # World to base_link transform
            t1 = TransformStamped()
            t1.header = Header()
            t1.header.frame_id = "world"
            t1.header.stamp.sec = sec
            t1.header.stamp.nsec = nsec
            t1.child_frame_id = "base_link"
            
            t1.transform = Transform()
            t1.transform.translation = Vector3()
            t1.transform.rotation = Quaternion()
            
            # Move in a circle
            angle = now * 0.2  # Slow rotation
            radius = 2.0
            t1.transform.translation.x = radius * np.cos(angle)
            t1.transform.translation.y = radius * np.sin(angle)
            t1.transform.translation.z = 0.0
            
            # Rotate to face direction of travel
            t1.transform.rotation = quaternion_from_euler(0, 0, angle + np.pi/2)
            
            transforms.append(t1)
            
            # Base link to arm_link1 transform
            t2 = TransformStamped()
            t2.header = Header()
            t2.header.frame_id = "base_link"
            t2.header.stamp.sec = sec
            t2.header.stamp.nsec = nsec
            t2.child_frame_id = "arm_link1"
            
            t2.transform = Transform()
            t2.transform.translation = Vector3()
            t2.transform.rotation = Quaternion()
            
            # Arm link is 0.5 meters up from base and rotating
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.5
            
            # Arm link rotates relative to base
            arm_angle = now * 0.5  # Faster rotation
            t2.transform.rotation = quaternion_from_euler(0, arm_angle, 0)
            
            transforms.append(t2)
            
            # Broadcast transforms
            broadcaster.send_transforms(transforms)
            
            print(f"Sent transforms at time {sec}.{nsec}")
            
            # Sleep to maintain publishing rate
            time.sleep(1.0 / rate)
            
    except KeyboardInterrupt:
        print("Broadcaster stopped")

if __name__ == "__main__":
    main()

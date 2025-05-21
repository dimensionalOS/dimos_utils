#!/usr/bin/env python3
"""
Robot Link Broadcaster

This script publishes a sample robot kinematic chain with transforms that match
the structure expected by robot_link_lookup_test.
It creates frames: world -> base_link -> link1 -> link2 -> ... -> link6
"""

import os
import sys
import time
import signal
import threading
import math
import numpy as np
from datetime import datetime

# Add necessary paths for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)  # Add the tf_lcm/python directory
sys.path.append(os.path.dirname(parent_dir))  # Add the tf_lcm parent

# Import the tf_lcm_py module
import tf_lcm_py

# Import lcm_msgs
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header

# Signal handling for clean shutdown
keep_running = True

def signal_handler(signum, frame):
    """Handle Ctrl+C to exit cleanly"""
    global keep_running
    print("Received signal {}, shutting down...".format(signum))
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

def lcm_handler_thread(lcm_instance):
    """Thread function to handle LCM messages"""
    global keep_running
    while keep_running:
        # Process LCM messages with a timeout
        lcm_instance.handle_timeout(100)  # 100ms timeout

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

def create_transform(frame_id, child_frame_id, x=0.0, y=0.0, z=0.0, 
                     roll=0.0, pitch=0.0, yaw=0.0):
    """Create a transform message with the given parameters"""
    transform = TransformStamped()
    transform.header = Header()
    transform.header.frame_id = frame_id
    
    # Set current time
    now = time.time()
    transform.header.stamp.sec = int(now)
    transform.header.stamp.nsec = int((now - int(now)) * 1e9)
    
    transform.child_frame_id = child_frame_id
    
    transform.transform = Transform()
    transform.transform.translation = Vector3()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    
    transform.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
    
    return transform

def main():
    """Main function to publish robot link transforms"""
    print("=== Robot Link Transform Broadcaster ===")
    print("Publishing robot link transforms that match the C++ test")
    print("Press Ctrl+C to exit")
    
    # Create LCM instance
    lcm = tf_lcm_py.LCM()
    if not lcm.good():
        print("ERROR: Failed to initialize LCM!")
        return 1
    
    # Start a thread to handle LCM messages
    lcm_thread = threading.Thread(target=lcm_handler_thread, args=(lcm,))
    lcm_thread.daemon = True
    lcm_thread.start()
    
    # Create a broadcaster
    broadcaster = tf_lcm_py.TransformBroadcaster()
    
    # Create static broadcaster for fixed frames
    static_broadcaster = tf_lcm_py.StaticTransformBroadcaster()
    
    # Publish the robot kinematic chain frames including all frames from the C++ test
    
    # First, publish static frames
    static_frames = [
        create_transform("world", "base_center", 0.0, 0.0, 0.0),
        create_transform("base_center", "base_link", 0.0, 0.0, 0.15),
        create_transform("base_link", "pillar_platform", 0.0, 0.0, 0.1),
        create_transform("base_link", "devkit_base_link", 0.1, 0.0, 0.05),
        create_transform("base_link", "pan_tilt_base", -0.1, 0.0, 0.05),
        create_transform("pan_tilt_base", "pan_tilt_pan", 0.0, 0.0, 0.05),
        create_transform("pan_tilt_pan", "pan_tilt_head", 0.0, 0.0, 0.1),
        create_transform("base_link", "piper_angled_mount", 0.0, 0.2, 0.1),
    ]
    
    # Send static transforms once
    static_broadcaster.send_transforms(static_frames)
    print(f"Published {len(static_frames)} static transforms")
    
    # Main loop to publish dynamic transforms
    try:
        while keep_running:
            # Calculate robot link positions with some animation
            current_time = time.time()
            theta = current_time * 0.3  # Slow rotation
            
            # Create dynamic transforms for robot arms
            transforms = [
                create_transform("pillar_platform", "link1", 0.0, 0.0, 0.15, 0, 0, theta),
                create_transform("link1", "link2", 0.0, 0.0, 0.2, 0, math.sin(theta)*0.2, 0),
                create_transform("link2", "link3", 0.0, 0.0, 0.25, 0, 0, -theta*0.5),
                create_transform("link3", "link4", 0.0, 0.0, 0.25, math.sin(theta)*0.1, 0, 0),
                create_transform("link4", "link5", 0.0, 0.0, 0.15, 0, math.sin(theta)*0.3, 0),
                create_transform("link5", "link6", 0.0, 0.0, 0.125, 0, 0, theta*0.25),
                create_transform("link6", "link7", 0.0, 0.0, 0.1),
                create_transform("link7", "link8", 0.0, 0.0, 0.05),
                create_transform("link8", "gripper_base", 0.0, 0.0, 0.05),
            ]
            
            # Send transforms
            broadcaster.send_transforms(transforms)
            
            # Sleep to maintain a reasonable rate
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Exiting due to keyboard interrupt")
    finally:
        print("Broadcaster stopped")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

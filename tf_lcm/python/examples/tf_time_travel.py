#!/usr/bin/env python3
"""
Example of the time travel functionality in tf_lcm
This demonstrates looking up transforms at different points in time
and using advanced time travel features.
"""

import time
import datetime
import sys
import os
import lcm
import numpy as np
from threading import Thread

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

def create_transform(parent_frame, child_frame, x, y, z, roll=0, pitch=0, yaw=0, time_offset=0):
    """
    Helper function to create a transform
    """
    now = time.time() + time_offset
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    
    transform = TransformStamped()
    transform.header = Header()
    transform.header.frame_id = parent_frame
    transform.header.stamp.sec = sec
    transform.header.stamp.nsec = nsec
    transform.child_frame_id = child_frame
    
    transform.transform = Transform()
    transform.transform.translation = Vector3()
    transform.transform.rotation = Quaternion()
    
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
    
    return transform

def main():
    print("TF_LCM Time Travel Example")
    
    # Create a buffer and add transforms directly to it
    buffer = tf_lcm_py.Buffer(cache_time=60.0)  # 60 second buffer
    
    # Store the current time
    now = time.time()
    
    # Create timestamps for different points in time
    past_2s = now - 2.0
    past_1s = now - 1.0
    future_1s = now + 1.0
    future_2s = now + 2.0
    
    # Create transforms at different times showing a robot moving
    
    # Robot at t-2s: Position (0,0,0)
    transform_past_2s = create_transform(
        "map", "robot", 0.0, 0.0, 0.0, 0, 0, 0, time_offset=-2.0
    )
    buffer.set_transform(transform_past_2s, "time_travel_example")
    
    # Robot at t-1s: Position (1,0,0)
    transform_past_1s = create_transform(
        "map", "robot", 1.0, 0.0, 0.0, 0, 0, 0, time_offset=-1.0
    )
    buffer.set_transform(transform_past_1s, "time_travel_example")
    
    # Robot at current time: Position (2,0,0)
    transform_now = create_transform(
        "map", "robot", 2.0, 0.0, 0.0, 0, 0, 0, time_offset=0.0
    )
    buffer.set_transform(transform_now, "time_travel_example")
    
    # Robot at t+1s: Position (3,0,0)
    transform_future_1s = create_transform(
        "map", "robot", 3.0, 0.0, 0.0, 0, 0, 0, time_offset=1.0
    )
    buffer.set_transform(transform_future_1s, "time_travel_example")
    
    # Robot at t+2s: Position (4,0,0)
    transform_future_2s = create_transform(
        "map", "robot", 4.0, 0.0, 0.0, 0, 0, 0, time_offset=2.0
    )
    buffer.set_transform(transform_future_2s, "time_travel_example")
    
    # Add a static transform for a sensor on the robot
    sensor_transform = create_transform(
        "robot", "sensor", 0.0, 0.5, 0.0, 0, 0, 0
    )
    buffer.set_transform(sensor_transform, "time_travel_example", is_static=True)
    
    print("Added transforms at different points in time:")
    print(f"  t-2s: Robot at (0,0,0)")
    print(f"  t-1s: Robot at (1,0,0)")
    print(f"  t=now: Robot at (2,0,0)")
    print(f"  t+1s: Robot at (3,0,0)")
    print(f"  t+2s: Robot at (4,0,0)")
    print(f"  Static: Sensor is 0.5m to the left of the robot")
    
    # Now demonstrate time travel capabilities
    
    # 1. Look up transforms at different times
    print("\nLooking up transforms at different times:")
    
    # Convert unix timestamps to datetime objects for the lookups
    dt_past_2s = datetime.datetime.fromtimestamp(past_2s)
    dt_past_1s = datetime.datetime.fromtimestamp(past_1s)
    dt_now = datetime.datetime.fromtimestamp(now)
    dt_future_1s = datetime.datetime.fromtimestamp(future_1s)
    dt_future_2s = datetime.datetime.fromtimestamp(future_2s)
    
    # Look up transforms at each time point
    try:
        # Past time points
        transform = buffer.lookup_transform("map", "robot", dt_past_2s)
        print(f"  Robot at t-2s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        transform = buffer.lookup_transform("map", "robot", dt_past_1s)
        print(f"  Robot at t-1s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        # Current time
        transform = buffer.lookup_transform("map", "robot", dt_now)
        print(f"  Robot at t=now: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        # Future time points
        transform = buffer.lookup_transform("map", "robot", dt_future_1s)
        print(f"  Robot at t+1s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        transform = buffer.lookup_transform("map", "robot", dt_future_2s)
        print(f"  Robot at t+2s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
    except tf_lcm_py.TransformException as e:
        print(f"Exception: {str(e)}")
    
    # 2. Advanced: Look up sensor position at different times
    print("\nLooking up sensor position at different times:")
    
    try:
        # Past
        transform = buffer.lookup_transform("map", "sensor", dt_past_2s)
        print(f"  Sensor at t-2s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        # Current
        transform = buffer.lookup_transform("map", "sensor", dt_now)
        print(f"  Sensor at t=now: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        
        # Future
        transform = buffer.lookup_transform("map", "sensor", dt_future_2s)
        print(f"  Sensor at t+2s: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
    
    except tf_lcm_py.TransformException as e:
        print(f"Exception: {str(e)}")
    
    # 3. Advanced: Use the time travel feature (transform from one time to another)
    print("\nAdvanced time travel (transform between different time points):")
    
    try:
        # Transform a point from the sensor frame at t-2s to the robot frame at t+2s
        transform = buffer.lookup_transform_with_fixed_frame(
            "robot", dt_future_2s,
            "sensor", dt_past_2s,
            "map"  # Fixed frame
        )
        
        print(f"  Sensor at t-2s to Robot at t+2s:")
        print(f"    Translation: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
        print(f"    Rotation: ({transform.transform.rotation.x}, "
              f"{transform.transform.rotation.y}, {transform.transform.rotation.z}, "
              f"{transform.transform.rotation.w})")
        
    except tf_lcm_py.TransformException as e:
        print(f"Exception in time travel lookup: {str(e)}")
    
    print("\nTime travel example completed")

if __name__ == "__main__":
    main()

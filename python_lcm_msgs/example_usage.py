#!/usr/bin/env python3
"""
Example showing how to use the LCM ROS message types with the lcm_msgs prefix.
"""

import lcm
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.geometry_msgs import Pose, Vector3
from lcm_msgs.std_msgs import Header

def main():
    # Initialize LCM
    lc = lcm.LCM()

    # Create a joint state message
    js = JointState()
    js.header = Header()
    js.name = ["joint1", "joint2", "joint3"]
    js.position = [0.1, 0.2, 0.3]
    js.velocity = [0.0, 0.0, 0.0]
    js.effort = []
    
    # Set length fields
    js.name_length = len(js.name)
    js.position_length = len(js.position)
    js.velocity_length = len(js.velocity)
    js.effort_length = len(js.effort)

    # Publish the message
    lc.publish("JOINT_STATE", js.encode())
    print(f"Published joint state with {js.name_length} joints")

    # Create a pose message
    pose = Pose()
    pose.position = Vector3()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    
    # Print the pose
    print(f"Pose position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Test script to verify that LCM-generated message types can be imported
without conflicts with ROS packages.
"""

import sys
print(f"Python version: {sys.version}")
print(f"Python path: {sys.path}")

# Test importing from lcm_msgs namespace
print("\nTesting lcm_msgs imports:")
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.geometry_msgs import Pose
from lcm_msgs.std_msgs import String

print("  ✓ JointState imported successfully")
print("  ✓ Pose imported successfully")
print("  ✓ String imported successfully")

# Create and print a simple message
js = JointState()
js.name = ["joint1", "joint2"]
js.position = [0.1, 0.2]
js.name_length = len(js.name)
js.position_length = len(js.position)

print(f"\nCreated JointState message with {js.name_length} joints: {js.name}")
print(f"Positions: {js.position}")

print("\nAll tests passed!")
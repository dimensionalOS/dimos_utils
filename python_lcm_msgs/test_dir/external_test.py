#!/usr/bin/env python3
"""
Test that the package works correctly from a different directory.
"""
import sys
print(f"Python path: {sys.path}")

import lcm_msgs
from lcm_msgs.geometry_msgs import Twist, Vector3
from lcm_msgs.std_msgs import Header

# Create a Twist message
twist = Twist()
twist.linear.x = 1.0
twist.linear.y = 2.0
twist.linear.z = 3.0
twist.angular.x = 0.1
twist.angular.y = 0.2
twist.angular.z = 0.3

# Print the message values to verify it worked
print("\nTwist message created successfully!")
print(f"Linear: ({twist.linear.x}, {twist.linear.y}, {twist.linear.z})")
print(f"Angular: ({twist.angular.x}, {twist.angular.y}, {twist.angular.z})")

# Try creating a Vector3 directly
vec = Vector3()
vec.x = 10.0
vec.y = 20.0
vec.z = 30.0
print(f"\nVector3: ({vec.x}, {vec.y}, {vec.z})")
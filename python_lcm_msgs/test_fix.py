#!/usr/bin/env python3
"""
Test script to verify our fix for the import issues.
This tests importing and creating a Twist object, which was failing before.
"""

# Import the package
import lcm_msgs
from lcm_msgs.geometry_msgs import Twist

# Create a Twist message
twist = Twist()
twist.linear.x = 1.0
twist.linear.y = 2.0
twist.linear.z = 3.0
twist.angular.x = 0.1
twist.angular.y = 0.2
twist.angular.z = 0.3

# Print the message values to verify it worked
print("Twist message created successfully!")
print(f"Linear: ({twist.linear.x}, {twist.linear.y}, {twist.linear.z})")
print(f"Angular: ({twist.angular.x}, {twist.angular.y}, {twist.angular.z})")
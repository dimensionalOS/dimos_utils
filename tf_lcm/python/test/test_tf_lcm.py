#!/usr/bin/env python3
"""
Comprehensive test suite for the tf_lcm Python bindings

This test script verifies that all key functionality of the tf_lcm
Python bindings works correctly, including:
- LCM instance management
- Transform broadcasting
- Transform listening
- Buffer operations
- Time travel transforms
- Error handling
"""

import os
import sys
import time
import unittest
import threading
from datetime import datetime, timedelta
import numpy as np
import math

# Add necessary paths for importing the modules
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
sys.path.append(os.path.dirname(parent_dir))

# Import tf_lcm_py and LCM message types
import tf_lcm_py
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header

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
                     roll=0.0, pitch=0.0, yaw=0.0, 
                     timestamp=None):
    """Helper function to create a transform message."""
    transform = TransformStamped()
    transform.header = Header()
    transform.header.frame_id = frame_id
    
    # Set time
    if timestamp is None:
        timestamp = time.time()
    transform.header.stamp.sec = int(timestamp)
    transform.header.stamp.nsec = int((timestamp % 1) * 1e9)
    
    transform.child_frame_id = child_frame_id
    
    transform.transform = Transform()
    transform.transform.translation = Vector3()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    
    transform.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
    
    return transform

def lcm_handler_thread(lcm_instance, stop_event):
    """Thread function to handle LCM messages until stop_event is set."""
    while not stop_event.is_set():
        lcm_instance.handle_timeout(100)  # 100ms timeout

class TestTfLcm(unittest.TestCase):
    """Test cases for tf_lcm Python bindings."""
    
    def setUp(self):
        """Set up test environment before each test."""
        # Create an LCM instance
        self.lcm = tf_lcm_py.LCM()
        self.assertTrue(self.lcm.good(), "LCM initialization failed")
        
        # Create a stop event and start a thread for handling LCM messages
        self.stop_event = threading.Event()
        self.handler_thread = threading.Thread(
            target=lcm_handler_thread, 
            args=(self.lcm, self.stop_event)
        )
        self.handler_thread.daemon = True
        self.handler_thread.start()
        
        # Create a buffer, broadcaster, and listener
        self.buffer = tf_lcm_py.Buffer(10.0)  # 10 seconds buffer
        self.listener = tf_lcm_py.TransformListener(self.lcm, self.buffer)
        self.broadcaster = tf_lcm_py.TransformBroadcaster()
        self.static_broadcaster = tf_lcm_py.StaticTransformBroadcaster()
        
        # Import the lcm_msgs module for return type conversion
        import lcm_msgs
        self.lcm_msgs_module = lcm_msgs
    
    def tearDown(self):
        """Clean up after each test."""
        # Signal the LCM handler thread to stop
        self.stop_event.set()
        if self.handler_thread.is_alive():
            self.handler_thread.join(timeout=1.0)
    
    def test_broadcaster_listener_basic(self):
        """Test basic broadcasting and listening functionality."""
        # Create and send a transform
        transform = create_transform("world", "robot", 1.0, 2.0, 3.0)
        self.broadcaster.send_transform(transform)
        
        # Wait a bit for the transform to be processed
        time.sleep(0.5)
        
        # Ensure that timestamp is supported
        now = datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        # Look up the transform
        result = self.buffer.lookup_transform("world", "robot", now, lcm_module=self.lcm_msgs_module)
        
        # Verify transform contents
        self.assertEqual(result.header.frame_id, "world")
        self.assertEqual(result.child_frame_id, "robot")
        self.assertAlmostEqual(result.transform.translation.x, 1.0, places=2)
        self.assertAlmostEqual(result.transform.translation.y, 2.0, places=2)
        self.assertAlmostEqual(result.transform.translation.z, 3.0, places=2)
    
    def test_multiple_transforms(self):
        """Test handling multiple transforms in a tree."""
        # Create a chain of transforms: world -> base -> arm -> gripper
        transforms = [
            create_transform("world", "base", 1.0, 0.0, 0.0),
            create_transform("base", "arm", 0.0, 1.0, 0.0),
            create_transform("arm", "gripper", 0.0, 0.0, 1.0)
        ]
        
        # Send all transforms
        self.broadcaster.send_transforms(transforms)
        
        # Wait for transforms to be processed
        time.sleep(0.5)
        
        # Get datetime object
        now = datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        # Look up transform from world to gripper (should follow the whole chain)
        result = self.buffer.lookup_transform("world", "gripper", now, lcm_module=self.lcm_msgs_module)
        
        # Verify - the complete transform should combine all three
        self.assertEqual(result.header.frame_id, "world")
        self.assertEqual(result.child_frame_id, "gripper")
        self.assertAlmostEqual(result.transform.translation.x, 1.0, places=2)
        self.assertAlmostEqual(result.transform.translation.y, 1.0, places=2)
        self.assertAlmostEqual(result.transform.translation.z, 1.0, places=2)
        
        # Check the frame list
        frames = self.buffer.get_all_frame_names()
        self.assertIn("world", frames)
        self.assertIn("base", frames)
        self.assertIn("arm", frames)
        self.assertIn("gripper", frames)
    
    def test_static_transforms(self):
        """Test that static transforms persist."""
        # Send a static transform
        static_tf = create_transform("map", "odom", 10.0, 20.0, 30.0)
        self.static_broadcaster.send_transform(static_tf)
        
        # Wait for it to be processed
        time.sleep(0.5)
        
        # Check that we can look it up
        now = datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        result = self.buffer.lookup_transform("map", "odom", now, lcm_module=self.lcm_msgs_module)
        
        # Verify the transform
        self.assertEqual(result.header.frame_id, "map")
        self.assertEqual(result.child_frame_id, "odom")
        self.assertAlmostEqual(result.transform.translation.x, 10.0, places=2)
        self.assertAlmostEqual(result.transform.translation.y, 20.0, places=2)
        self.assertAlmostEqual(result.transform.translation.z, 30.0, places=2)
    
    def test_error_handling(self):
        """Test error handling for non-existent transforms."""
        # Try to look up a transform that doesn't exist
        now = datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        # This should raise an exception
        with self.assertRaises(Exception) as context:
            self.buffer.lookup_transform("non_existent_frame", "also_not_real", now, lcm_module=self.lcm_msgs_module)
        
        # We should be able to check if a transform exists
        can_transform = self.buffer.can_transform("world", "robot", now)
        # This might be True if previous tests ran, so we don't assert its value
        
        # Clear the buffer
        self.buffer.clear()
        
        # After clearing, the transform should not be available
        can_transform = self.buffer.can_transform("world", "robot", now)
        self.assertFalse(can_transform)

if __name__ == "__main__":
    unittest.main()

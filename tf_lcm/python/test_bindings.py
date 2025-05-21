#!/usr/bin/env python3
"""
Test script to verify that the Python bindings for tf_lcm work properly
"""
import sys
import os
import time
import threading
from datetime import datetime

# Add the parent directory to the path so we can import the module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import tf_lcm module
import tf_lcm_py

# Import LCM message types
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header

def create_test_transform(frame_id, child_frame_id):
    """Create a test transform"""
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
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    
    transform.transform.rotation = Quaternion()
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    
    return transform

def lcm_handler_thread(lcm_instance):
    """Thread function to handle LCM messages"""
    try:
        while True:
            # Handle LCM messages with a timeout of 100ms
            lcm_instance.handle_timeout(100)
    except Exception as e:
        print(f"LCM handler thread error: {e}")

def main():
    print("Testing tf_lcm Python bindings")
    
    # Create an LCM instance
    lcm_instance = tf_lcm_py.LCM()
    if not lcm_instance.good():
        print("Failed to initialize LCM")
        return 1
    
    # Start a thread to handle LCM messages in the background
    handler_thread = threading.Thread(target=lcm_handler_thread, args=(lcm_instance,))
    handler_thread.daemon = True
    handler_thread.start()
    
    print("Step 1: Create buffer and listener")
    buffer = tf_lcm_py.Buffer(10.0)  # 10 seconds buffer
    listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
    
    print("Step 2: Create broadcaster")
    broadcaster = tf_lcm_py.TransformBroadcaster()
    
    # Create a test transform
    transform = create_test_transform("world", "base_link")
    print(f"Created test transform from {transform.header.frame_id} to {transform.child_frame_id}")
    
    print("Step 3: Send transform")
    try:
        broadcaster.send_transform(transform)
        print("Transform sent successfully")
    except Exception as e:
        print(f"Error sending transform: {e}")
        return 1
    
    # Wait a bit for the transform to be processed
    print("Waiting for transform to be processed...")
    time.sleep(1.0)
    
    # Try to look up the transform
    print("Step 4: Look up transform")
    now = datetime.now()
    # Add timestamp method if it doesn't exist
    if not hasattr(now, 'timestamp'):
        now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
    
    # Import the lcm_msgs module for type conversions
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import lcm_msgs
    
    try:
        # Pass the lcm_msgs module explicitly to help with type conversion
        result = buffer.lookup_transform("world", "base_link", now, lcm_module=lcm_msgs)
        print("Successfully looked up transform:")
        print(f"Translation: ({result.transform.translation.x}, {result.transform.translation.y}, {result.transform.translation.z})")
        print(f"Rotation: ({result.transform.rotation.x}, {result.transform.rotation.y}, {result.transform.rotation.z}, {result.transform.rotation.w})")
        
        # Get all frame names
        frames = buffer.get_all_frame_names()
        print(f"All frames: {frames}")
        
        print("Test passed!")
        return 0
    except Exception as e:
        print(f"Error looking up transform: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())

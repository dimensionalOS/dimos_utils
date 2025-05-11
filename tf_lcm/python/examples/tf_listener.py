#!/usr/bin/env python3
"""
Example of listening for transforms using the tf_lcm Python bindings

This example demonstrates:
1. Setting up an LCM instance for tf_lcm
2. Creating a Buffer and TransformListener
3. Running LCM message handling in a background thread
4. Looking up transforms between coordinate frames
5. Querying the transform tree structure

Run this example alongside the broadcaster example to see it working.
"""

import time
import datetime
import sys
import os
import threading
import numpy as np
from datetime import datetime, timedelta

# Make sure lcm_msgs is in the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import lcm_msgs

# Add the parent directory to the path so we can import the module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import tf_lcm module (assumes the library has been built and installed)
import tf_lcm_py

def lcm_handler_thread(lcm_instance):
    """Thread function to handle LCM messages"""
    try:
        while True:
            # Handle LCM messages with a timeout of 100ms
            lcm_instance.handle_timeout(100)
    except KeyboardInterrupt:
        pass

def main():
    print("TF_LCM Listener Example")
    
    # Create an LCM instance
    lcm_instance = tf_lcm_py.LCM()
    
    if not lcm_instance.good():
        print("Failed to initialize LCM")
        return 1
    
    # Create a buffer
    buffer = tf_lcm_py.Buffer(10.0)  # 10 seconds buffer
    
    # Create a listener with the buffer and LCM instance
    listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
    
    # Start a thread to handle LCM messages in the background
    handler_thread = threading.Thread(target=lcm_handler_thread, args=(lcm_instance,))
    handler_thread.daemon = True  # Set as daemon so it exits when the main thread exits
    handler_thread.start()
    
    print("Listening for transforms. Run the broadcaster example in another terminal.")
    
    rate = 10  # Hz
    
    try:
        while True:
            # Get current time as a datetime object and convert to C++ compatible time
            now = datetime.now()  # Python datetime
            # Add timestamp method if it doesn't exist (for Python <3.3)
            if not hasattr(now, 'timestamp'):
                now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
            
            # Try to look up the transform from world to link6
            try:
                # Explicitly pass the lcm_msgs module to help with type conversion
                transform = buffer.lookup_transform("world", "link6", now, lcm_module=lcm_msgs)
                
                print("\nFound transform from world to link6:")
                print(f"  Translation: ({transform.transform.translation.x:.2f}, "
                      f"{transform.transform.translation.y:.2f}, "
                      f"{transform.transform.translation.z:.2f})")
                print(f"  Rotation: ({transform.transform.rotation.x:.2f}, "
                      f"{transform.transform.rotation.y:.2f}, "
                      f"{transform.transform.rotation.z:.2f}, "
                      f"{transform.transform.rotation.w:.2f})")
                
                # Also check if we can do a reverse lookup (link6 to world)
                can_reverse = buffer.can_transform("link6", "world", now)
                print(f"  Can transform from link6 to world: {can_reverse}")
                
                # Get all available frames
                frames = buffer.get_all_frame_names()
                print(f"  All frame names: {frames}")
                
            except tf_lcm_py.TransformException as e:
                print(f"Exception: {str(e)}")
            
            # Sleep to maintain rate
            time.sleep(1.0 / rate)
            
    except KeyboardInterrupt:
        print("Listener stopped")
        return 0

if __name__ == "__main__":
    main()

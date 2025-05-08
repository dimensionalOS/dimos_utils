#!/usr/bin/env python3
"""
Example of listening for transforms using the tf_lcm Python bindings
"""

import time
import datetime
import sys
import os
import lcm
import numpy as np

# Add the parent directory to the path so we can import the module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import tf_lcm module (assumes the library has been built and installed)
import tf_lcm_py

def main():
    print("TF_LCM Listener Example")
    
    # Create a listener (automatically creates a buffer)
    listener = tf_lcm_py.TransformListener()
    
    # Get the buffer from the listener
    buffer = listener.get_buffer()
    
    print("Listening for transforms. Run the broadcaster example in another terminal.")
    
    rate = 10  # Hz
    
    try:
        while True:
            # Get current time as a datetime object (used for tf lookups)
            now = datetime.datetime.now()
            
            # Try to look up the transform from world to arm_link1
            try:
                transform = buffer.lookup_transform("world", "arm_link1", now)
                
                print("\nFound transform from world to arm_link1:")
                print(f"  Translation: ({transform.transform.translation.x:.2f}, "
                      f"{transform.transform.translation.y:.2f}, "
                      f"{transform.transform.translation.z:.2f})")
                print(f"  Rotation: ({transform.transform.rotation.x:.2f}, "
                      f"{transform.transform.rotation.y:.2f}, "
                      f"{transform.transform.rotation.z:.2f}, "
                      f"{transform.transform.rotation.w:.2f})")
                
                # Also check if we can do a reverse lookup (arm_link1 to world)
                can_reverse = buffer.can_transform("arm_link1", "world", now)
                print(f"  Can transform from arm_link1 to world: {can_reverse}")
                
                # Get all available frames
                frames = buffer.get_all_frame_names()
                print(f"  All frame names: {frames}")
                
            except tf_lcm_py.TransformException as e:
                print(f"Exception: {str(e)}")
            
            # Sleep to maintain rate
            time.sleep(1.0 / rate)
            
    except KeyboardInterrupt:
        print("Listener stopped")

if __name__ == "__main__":
    main()

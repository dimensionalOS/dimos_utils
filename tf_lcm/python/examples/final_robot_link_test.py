#!/usr/bin/env python3
"""
Final Robot Link Lookup Test (Python)

This test correctly subscribes to the LCM channels using the same format as the C++ code:
- "tf#tf2_msgs.TFMessage" for regular transforms
- "tf_static#tf2_msgs.TFMessage" for static transforms

It also properly handles module imports and parameters for the tf_lcm_py bindings.
"""

import os
import sys
import time
import signal
import datetime
import lcm  # Native Python LCM library

# Import modules directly - no path manipulation needed anymore!
import tf_lcm_py

# Import LCM message types
import lcm_msgs
from lcm_msgs.tf2_msgs.TFMessage import TFMessage

# Signal handling for clean shutdown
keep_running = True

def signal_handler(signum, frame):
    global keep_running
    print(f"Received signal {signum}, shutting down...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

class TransformReceiver:
    def __init__(self):
        # Create a buffer for transforms
        self.buffer = tf_lcm_py.Buffer(30.0)  # 30 seconds cache
        
        # Create LCM instance
        self.lc = lcm.LCM()
        
        # Subscribe to TF channels using the EXACT same channel names as C++
        self.lc.subscribe("tf#tf2_msgs.TFMessage", self.tf_callback)
        self.lc.subscribe("tf_static#tf2_msgs.TFMessage", self.tf_static_callback)
        
        print("TransformReceiver initialized")
        print("Subscribed to 'tf#tf2_msgs.TFMessage' and 'tf_static#tf2_msgs.TFMessage'")
    
    def tf_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {msg.transforms_length} transforms on channel {channel}")
            
            # Process each transform
            for i in range(msg.transforms_length):
                transform = msg.transforms[i]
                print(f"  {transform.header.frame_id} -> {transform.child_frame_id}")
                self.buffer.set_transform(transform, "lcm_publisher", False)
        except Exception as e:
            print(f"Error processing transform: {e}")
    
    def tf_static_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {msg.transforms_length} static transforms on channel {channel}")
            
            # Process each transform
            for i in range(msg.transforms_length):
                transform = msg.transforms[i]
                print(f"  {transform.header.frame_id} -> {transform.child_frame_id}")
                self.buffer.set_transform(transform, "static_publisher", True)
        except Exception as e:
            print(f"Error processing static transform: {e}")

def print_transform(transform):
    """Print transform in the same format as the C++ test"""
    print("\nTransform found!")
    print(f"  Header frame: {transform.header.frame_id}")
    print(f"  Child frame: {transform.child_frame_id}")
    print(f"  Timestamp: {transform.header.stamp.sec}.{transform.header.stamp.nsec:09d}")
    print(f"  Translation: ({transform.transform.translation.x:.6f}, "
          f"{transform.transform.translation.y:.6f}, "
          f"{transform.transform.translation.z:.6f})")
    print(f"  Rotation (quaternion): ({transform.transform.rotation.w:.6f}, "
          f"{transform.transform.rotation.x:.6f}, "
          f"{transform.transform.rotation.y:.6f}, "
          f"{transform.transform.rotation.z:.6f})")

def main():
    print("=== Final Robot Link Transform Lookup Test (Python) ===")
    print("Listening for transforms between 'world' and 'link6'...")
    print("Press Ctrl+C to exit")
    
    # Define the source and target frames (same as C++ test)
    target_frame = "world"
    source_frame = "link6"
    
    # Create the transform receiver
    receiver = TransformReceiver()
    
    # Main lookup loop
    lookup_attempts = 0
    found_transform = False
    
    while keep_running:
        try:
            # Handle LCM messages
            receiver.lc.handle_timeout(100)  # 100ms timeout
            
            # Get current time
            now = datetime.datetime.now()
            
            # Try to look up the transform
            can_transform = receiver.buffer.can_transform(target_frame, source_frame, now)
            
            if can_transform:
                # Look up the transform without lcm_module parameter, as we're using set_transform directly
                transform = receiver.buffer.lookup_transform(target_frame, source_frame, now)
                print_transform(transform)
                found_transform = True
                
                # Get all frame names to help with debugging
                frames = receiver.buffer.get_all_frame_names()
                print(f"\nAll frames in buffer ({len(frames)} total):")
                for frame in sorted(frames):
                    print(f"  {frame}")
                
                # Test passed, we can exit
                print(f"\n✅ SUCCESS: Found transform between '{target_frame}' and '{source_frame}'!")
                break
            else:
                # Increment counter and report status every few attempts
                lookup_attempts += 1
                if lookup_attempts % 10 == 0:
                    print(f"Waiting for transform between '{target_frame}' and '{source_frame}' ({lookup_attempts} attempts)...")
                    
                    # Print out all frames we have received
                    frames = receiver.buffer.get_all_frame_names()
                    if frames:
                        print(f"Frames received so far ({len(frames)} total):")
                        for frame in sorted(frames):
                            print(f"  {frame}")
                    else:
                        print("No frames received yet")
                
                # Sleep briefly to avoid spinning too fast
                time.sleep(0.5)
            
            # Timeout after many attempts (60 seconds)
            if lookup_attempts > 120:
                print(f"\n❌ ERROR: Failed to find transform between '{target_frame}' and '{source_frame}' after {lookup_attempts} attempts!")
                break
                
        except Exception as e:
            print(f"Exception: {e}")
            import traceback
            traceback.print_exc()
            
            # Sleep to avoid spinning too fast
            time.sleep(0.5)
            
            # Increment attempts
            lookup_attempts += 1
            
            # Timeout after many attempts
            if lookup_attempts > 120:
                print(f"\n❌ ERROR: Failed to find transform between '{target_frame}' and '{source_frame}' after {lookup_attempts} attempts!")
                break
    
    # Final result
    if not found_transform:
        print(f"\n❌ Test FAILED: Could not find transform between '{target_frame}' and '{source_frame}'.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Direct LCM test for tf_lcm Python bindings

This test uses the native Python LCM library directly to handle message reception,
which should bypass any issues with our custom PyLCM wrapper class.
"""

import os
import sys
import time
import datetime
import lcm  # Native Python LCM library

# Add necessary paths for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)  # Add the tf_lcm/python directory
sys.path.append(os.path.dirname(parent_dir))  # Add the tf_lcm parent

# Import tf_lcm_py
import tf_lcm_py

# Import LCM message types
from lcm_msgs.geometry_msgs import TransformStamped
from lcm_msgs.std_msgs import Header
import lcm_msgs.tf2_msgs.TFMessage as TFMessage

class TransformListener:
    def __init__(self):
        # Initialize native Python LCM
        self.lc = lcm.LCM()
        
        # Create a buffer for transforms
        self.buffer = tf_lcm_py.Buffer(30.0)  # 30 seconds cache
        
        # Subscribe to TF channels
        self.lc.subscribe("tf", self.tf_callback)
        self.lc.subscribe("tf_static", self.tf_static_callback)
        
        print("TransformListener initialized with native Python LCM")
    
    def tf_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {len(msg.transforms)} transforms on channel {channel}")
            
            # Process each transform
            for tf in msg.transforms:
                print(f"  {tf.header.frame_id} -> {tf.child_frame_id}")
                self.buffer.set_transform(tf, "lcm_publisher", False)
        except Exception as e:
            print(f"Error processing transform: {e}")
    
    def tf_static_callback(self, channel, data):
        try:
            # Decode the TF message
            msg = TFMessage.decode(data)
            print(f"Received {len(msg.transforms)} static transforms on channel {channel}")
            
            # Process each transform
            for tf in msg.transforms:
                print(f"  {tf.header.frame_id} -> {tf.child_frame_id}")
                self.buffer.set_transform(tf, "static_publisher", True)
        except Exception as e:
            print(f"Error processing static transform: {e}")
    
    def lookup_transform(self, target_frame, source_frame):
        """Look up a transform between two frames"""
        # Get current time
        now = datetime.datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        try:
            # Check if transform is available
            if self.buffer.can_transform(target_frame, source_frame, now):
                # Look up transform
                transform = self.buffer.lookup_transform(target_frame, source_frame, now, lcm_module=lcm_msgs)
                return transform
        except Exception as e:
            print(f"Error looking up transform: {e}")
        
        return None
    
    def get_frames(self):
        """Get all available frames"""
        return self.buffer.get_all_frame_names()

def main():
    print("=== Direct LCM TF Listener Test ===")
    print("Using native Python LCM for message handling")
    
    # Create our listener
    listener = TransformListener()
    
    # Define frames to look up
    target_frame = "world"
    source_frame = "link6"
    print(f"Looking for transforms between '{target_frame}' and '{source_frame}'...")
    
    # Main loop
    attempts = 0
    max_attempts = 60  # Run for about 30 seconds
    
    while attempts < max_attempts:
        # Handle incoming LCM messages
        listener.lc.handle_timeout(100)  # 100ms timeout
        
        # Try to look up our transform
        transform = listener.lookup_transform(target_frame, source_frame)
        
        if transform:
            print(f"\nFound transform from '{target_frame}' to '{source_frame}'!")
            print(f"  Translation: ({transform.transform.translation.x:.6f}, "
                  f"{transform.transform.translation.y:.6f}, "
                  f"{transform.transform.translation.z:.6f})")
            print(f"  Rotation: ({transform.transform.rotation.w:.6f}, "
                  f"{transform.transform.rotation.x:.6f}, "
                  f"{transform.transform.rotation.y:.6f}, "
                  f"{transform.transform.rotation.z:.6f})")
            
            # Show all frames
            frames = listener.get_frames()
            print(f"All frames ({len(frames)}):")
            for frame in sorted(frames):
                print(f"  {frame}")
            
            print("\n✅ SUCCESS: Test passed!")
            return 0
        
        # Status update every 10 attempts
        if attempts % 10 == 0:
            print(f"Waiting for transforms... (attempt {attempts+1})")
            frames = listener.get_frames()
            if frames:
                print(f"Frames received ({len(frames)}):")
                for frame in sorted(frames):
                    print(f"  {frame}")
            else:
                print("No frames received yet")
        
        attempts += 1
        time.sleep(0.5)
    
    print("\n❌ ERROR: No transforms found after 30 seconds")
    return 1

if __name__ == "__main__":
    sys.exit(main())

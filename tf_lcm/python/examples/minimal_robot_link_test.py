#!/usr/bin/env python3
"""
Minimal test for tf_lcm Python bindings

This is a stripped-down test that focuses solely on validating that we can
receive transforms between 'world' and 'link6' frames.
"""

import os
import sys
import time
from datetime import datetime

# Import modules directly - no path manipulation needed anymore!
import tf_lcm_py
import lcm_msgs

def main():
    print("=== Minimal Robot Link Transform Test ===")
    
    # Define frames to look up
    target_frame = "world"
    source_frame = "link6"
    
    # Create a Buffer instance
    buffer = tf_lcm_py.Buffer(30.0)
    
    # Create LCM instance
    lcm_instance = tf_lcm_py.LCM()
    
    # Create a TransformListener with our LCM instance and buffer
    listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
    
    print(f"Looking for transforms between '{target_frame}' and '{source_frame}'...")
    
    attempts = 0
    max_attempts = 120  # Same as C++ test
    
    while attempts < max_attempts:
        # Process LCM messages
        lcm_instance.handle_timeout(100)  # 100ms timeout
        
        # Get current time
        now = datetime.now()
        if not hasattr(now, 'timestamp'):
            now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
        
        # Check if we can find the transform
        if buffer.can_transform(target_frame, source_frame, now):
            print(f"Found transform between '{target_frame}' and '{source_frame}'!")
            
            # Look up the transform
            transform = buffer.lookup_transform(target_frame, source_frame, now, lcm_module=lcm_msgs)
            
            # Print details
            print(f"  Translation: ({transform.transform.translation.x:.6f}, "
                  f"{transform.transform.translation.y:.6f}, "
                  f"{transform.transform.translation.z:.6f})")
            print(f"  Rotation: ({transform.transform.rotation.w:.6f}, "
                  f"{transform.transform.rotation.x:.6f}, "
                  f"{transform.transform.rotation.y:.6f}, "
                  f"{transform.transform.rotation.z:.6f})")
            
            # Get all frames
            frames = buffer.get_all_frame_names()
            print(f"All frames in buffer ({len(frames)} total):")
            for frame in sorted(frames):
                print(f"  {frame}")
            
            print("\n✅ SUCCESS: Test passed!")
            return 0
        
        # Increment counter and report status every 10 attempts
        attempts += 1
        if attempts % 10 == 0:
            print(f"Still waiting... (attempt {attempts}/{max_attempts})")
            frames = buffer.get_all_frame_names()
            if frames:
                print(f"Frames received so far ({len(frames)} total):")
                for frame in sorted(frames):
                    print(f"  {frame}")
            else:
                print("No frames received yet")
        
        # Brief pause
        time.sleep(0.5)
    
    print(f"\n❌ ERROR: No transform found after {max_attempts} attempts")
    return 1

if __name__ == "__main__":
    sys.exit(main())

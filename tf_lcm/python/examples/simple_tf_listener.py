#!/usr/bin/env python3
"""
A very simple LCM-based tf listener for debugging

This script focuses on basic LCM functionality without complex features
to help debug the Python bindings for tf_lcm.
"""

import os
import sys
import time
from datetime import datetime

# Import tf_lcm_py and lcm_msgs
import tf_lcm_py
import lcm_msgs

def main():
    print("=== Simple TF Listener ===")
    
    # Use explicit UDP provider URL to avoid multicast issues
    lcm_url = "udpm://239.255.76.67:7667?ttl=1"
    print(f"Initializing LCM with URL: {lcm_url}")
    
    try:
        # Initialize LCM with explicit URL
        lcm_instance = tf_lcm_py.LCM(lcm_url)
        if not lcm_instance.good():
            print("ERROR: Failed to initialize LCM")
            return 1
        
        print("LCM initialized successfully")
            
        # Create a Buffer instance
        buffer = tf_lcm_py.Buffer(30.0)
        print("Buffer created")
        
        # Create a TransformListener with our LCM instance and buffer
        listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
        print("TransformListener created")
        
        # Main loop
        count = 0
        target_frame = "world"
        source_frame = "link6"
        
        print(f"Starting to listen for transforms between '{target_frame}' and '{source_frame}'...")
        
        while count < 60:  # Run for about 30 seconds
            # Process any LCM messages
            lcm_instance.handle_timeout(100)
            
            # Get current time
            now = datetime.now()
            if not hasattr(now, 'timestamp'):
                now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
            
            # Check for transform availability
            can_transform = buffer.can_transform(target_frame, source_frame, now)
            
            if can_transform:
                print(f"\nCan transform from '{target_frame}' to '{source_frame}'")
                
                # Look up the transform
                transform = buffer.lookup_transform(target_frame, source_frame, now, lcm_module=lcm_msgs)
                
                # Print details
                print(f"  Translation: ({transform.transform.translation.x:.6f}, "
                      f"{transform.transform.translation.y:.6f}, "
                      f"{transform.transform.translation.z:.6f})")
                
                # Get all frame names
                frames = buffer.get_all_frame_names()
                print(f"All frames ({len(frames)}):")
                for frame in sorted(frames):
                    print(f"  {frame}")
                
                # Success!
                print("\n✅ SUCCESS: Found transforms!")
                return 0
            
            # Report status every 10 iterations
            if count % 10 == 0:
                print(f"Waiting for transforms... (attempt {count+1})")
                frames = buffer.get_all_frame_names()
                print(f"Frames seen so far: {len(frames)}")
            
            count += 1
            time.sleep(0.5)
        
        print("\n❌ ERROR: No transforms found")
        return 1
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())

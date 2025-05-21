#!/usr/bin/env python3
"""
Python equivalent of the robot_link_lookup_test.cpp

This script listens for transforms between 'world' and 'link6' frames,
replicating the functionality and output format of the C++ test.

Simplified version without complex threading to avoid mutex issues.
"""

import os
import sys
import time
import signal
import threading
from datetime import datetime
import traceback

# Add necessary paths for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)  # Add the tf_lcm/python directory
sys.path.append(os.path.dirname(parent_dir))  # Add the tf_lcm parent

# Import the tf_lcm_py module
import tf_lcm_py

# Import the lcm_msgs module for type conversions
import lcm_msgs

# Signal handling for clean shutdown
keep_running = True

def signal_handler(signum, frame):
    """Handle Ctrl+C to exit cleanly"""
    global keep_running
    print("Received signal {}, shutting down...".format(signum))
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

def lcm_handler_thread(lcm_instance):
    """Thread function to handle LCM messages"""
    global keep_running
    while keep_running:
        # Process LCM messages with a timeout
        lcm_instance.handle_timeout(100)  # 100ms timeout

def print_transform(transform):
    """Print transform information in the same format as the C++ test"""
    print("Transform found!")
    print("  Header frame: {}".format(transform.header.frame_id))
    print("  Child frame: {}".format(transform.child_frame_id))
    print("  Timestamp: {}.{:09d}".format(
        transform.header.stamp.sec, 
        transform.header.stamp.nsec))
    print("  Translation: ({}, {}, {})".format(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z))
    print("  Rotation (quaternion): ({}, {}, {}, {})".format(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z))

def main():
    """Main function that replicates the C++ test logic"""
    print("=== Robot Link Transform Lookup Test ===")
    print("Listening for transforms between 'world' and 'link6'...")
    print("Press Ctrl+C to exit")
    
    # Define source and target frames (same as C++ test)
    target_frame = "world"
    source_frame = "link6"
    
    # Create LCM instance
    lcm = tf_lcm_py.LCM()
    if not lcm.good():
        print("ERROR: Failed to initialize LCM!")
        return 1
    
    # Create buffer with 30 seconds cache time (same as C++ test)
    buffer = tf_lcm_py.Buffer(30.0)
    
    # Create transform listener
    listener = tf_lcm_py.TransformListener(lcm, buffer)
    
    # We'll handle LCM messages directly in the main loop instead of using a separate thread
    # This avoids potential threading/mutex issues with the Python bindings
    
    # Main lookup loop
    lookup_attempts = 0
    found_transform = False
    
    while keep_running:
        try:
            # Process any incoming LCM messages first (directly in main loop)
            lcm.handle_timeout(100)  # 100ms timeout, similar to C++ version
            
            # Get current time
            now = datetime.now()
            # Add timestamp method if it doesn't exist
            if not hasattr(now, 'timestamp'):
                now.timestamp = lambda: time.mktime(now.timetuple()) + now.microsecond / 1e6
            
            # Check if we can transform
            can_transform = buffer.can_transform(target_frame, source_frame, now)
            
            if can_transform:
                # Look up the transform
                transform = buffer.lookup_transform(target_frame, source_frame, now, lcm_module=lcm_msgs)
                print_transform(transform)
                found_transform = True
                
                # Get all frame names to help with debugging
                frames = buffer.get_all_frame_names()
                print("All frames in buffer ({} total):".format(len(frames)))
                for frame in sorted(frames):
                    print("  {}".format(frame))
                
                # Test passed, we can exit
                print("\n✅ SUCCESS: Found transform between '{}' and '{}'!".format(
                    target_frame, source_frame))
                break
            else:
                # Increment counter and report status every few attempts
                lookup_attempts += 1
                if lookup_attempts % 10 == 0:
                    print("Waiting for transform between '{}' and '{}' ({} attempts)...".format(
                        target_frame, source_frame, lookup_attempts))
                    
                    # Print out all frames we have received
                    frames = buffer.get_all_frame_names()
                    print("Frames received so far ({} total):".format(len(frames)))
                    for frame in sorted(frames):
                        print("  {}".format(frame))
                
                # Sleep briefly to avoid spinning too fast
                time.sleep(0.5)
            
            # Timeout after many attempts (60 seconds, same as C++)
            if lookup_attempts > 120:
                print("\n❌ ERROR: Failed to find transform between '{}' and '{}' after {} attempts!".format(
                    target_frame, source_frame, lookup_attempts))
                break
                
        except Exception as e:
            lookup_attempts += 1
            if lookup_attempts % 10 == 0:
                print("Exception: {}".format(str(e)))
                
                # Print out all frames we have received to help with debugging
                frames = buffer.get_all_frame_names()
                print("Frames received so far ({} total):".format(len(frames)))
                for frame in sorted(frames):
                    print("  {}".format(frame))
            
            # Sleep to avoid spinning too fast
            time.sleep(0.5)
            
            # Timeout after many attempts (60 seconds)
            if lookup_attempts > 120:
                print("\n❌ ERROR: Failed to find transform between '{}' and '{}' after {} attempts!".format(
                    target_frame, source_frame, lookup_attempts))
                break
    
    # Final result
    if not found_transform:
        print("\n❌ Test FAILED: Could not find transform between '{}' and '{}'.".format(
            target_frame, source_frame))
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Minimal test for tf_lcm Python bindings

This is a stripped-down test that focuses solely on validating that we can
receive transforms between 'world' and 'link6' frames.
"""

import os
import sys
import time
import signal
import atexit
from datetime import datetime
from contextlib import contextmanager

# Import modules directly - no path manipulation needed anymore!
import tf_lcm_py
import lcm_msgs

# Global variables to hold resources that need cleanup
_resources_to_cleanup = []

@contextmanager
def safe_lcm_instance():
    """Context manager for safely managing LCM instance lifecycle"""
    # Try multiple provider URLs to find one that works
    provider_urls = [
        None,  # Default
        "udpm://localhost:7667",
        "file:///tmp/lcm-log.data",
        "memq://"
    ]
    
    lcm_instance = None
    for provider in provider_urls:
        try:
            if provider is None:
                lcm_instance = tf_lcm_py.LCM()
            else:
                lcm_instance = tf_lcm_py.LCM(provider)
            
            if lcm_instance.good():
                print(f"Successfully initialized LCM with provider: {provider if provider else 'default'}")
                break
            else:
                print(f"LCM initialized but not in good state with provider: {provider if provider else 'default'}")
                lcm_instance = None
        except Exception as e:
            print(f"Failed to initialize LCM with provider {provider}: {e}")
    
    if lcm_instance is None:
        print("WARNING: Could not initialize any LCM provider, using fallback")
        lcm_instance = tf_lcm_py.LCM("null://")
    
    try:
        yield lcm_instance
    finally:
        # The context manager ensures we keep a reference to lcm_instance until the end
        # This helps prevent premature garbage collection
        pass

def cleanup_resources():
    """Clean up resources before exiting"""
    global _resources_to_cleanup
    
    # Force cleanup of resources in reverse order (last created first)
    for resource in reversed(_resources_to_cleanup):
        try:
            # For objects like TransformListener that might have a close or shutdown method
            if hasattr(resource, 'close'):
                resource.close()
            elif hasattr(resource, 'shutdown'):
                resource.shutdown()
            
            # Explicitly delete the resource
            del resource
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    # Clear the resources list
    _resources_to_cleanup = []

def signal_handler(sig, frame):
    """Handle signals like CTRL+C by cleaning up resources first"""
    print("\nInterrupt received, cleaning up...")
    cleanup_resources()
    sys.exit(1)

def main():
    global _resources_to_cleanup
    
    # Set up signal handlers for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup on normal exit
    atexit.register(cleanup_resources)
    
    print("=== Minimal Robot Link Transform Test ===")
    
    # Define frames to look up
    target_frame = "world"
    source_frame = "link6"
    
    # Create resources in a specific order and track them for cleanup
    buffer = tf_lcm_py.Buffer(30.0)
    _resources_to_cleanup.append(buffer)
    
    # Use context manager for LCM instance
    with safe_lcm_instance() as lcm_instance:
        _resources_to_cleanup.append(lcm_instance)
        
        # Create a TransformListener with our LCM instance and buffer
        listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
        _resources_to_cleanup.append(listener)
        
        print(f"Looking for transforms between '{target_frame}' and '{source_frame}'...")
        
        attempts = 0
        max_attempts = 120  # Same as C++ test
        
        while attempts < max_attempts:
            try:
                # Process LCM messages with error handling
                if not lcm_instance.handle_timeout(100):  # 100ms timeout
                    # If handle_timeout returns false, we might need to re-check if LCM is still good
                    if not lcm_instance.good():
                        print("WARNING: LCM instance is no longer in a good state")
                
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
                
            except Exception as e:
                print(f"Error during main loop: {e}")
                attempts += 1
                time.sleep(1)  # Longer pause after an error
        
        print(f"\n❌ ERROR: No transform found after {max_attempts} attempts")
        return 1

if __name__ == "__main__":
    try:
        exit_code = main()
        # Explicit cleanup before exiting
        cleanup_resources()
        sys.exit(exit_code)
    except Exception as e:
        print(f"Unhandled exception: {e}")
        # Make sure we clean up even if an exception occurred
        cleanup_resources()
        sys.exit(1)

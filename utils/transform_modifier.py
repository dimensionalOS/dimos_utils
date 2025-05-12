#!/usr/bin/env python3
"""
Transform Modifier for Robot Arm Control

This script:
1. Subscribes to cmd_pos_diff Twist/TwistStamped messages
2. Looks up the current transform between 'world' and 'link6' frames
3. Applies the Twist/TwistStamped changes to the transform
4. Publishes the resulting position to cmd_pos topic
"""

import os
import sys
import time
import signal
import atexit
import argparse
from datetime import datetime
from contextlib import contextmanager

# Import modules
import tf_lcm_py
import lcm

# Try to import LCM message types - do this dynamically to handle import errors gracefully
try:
    from lcm_msgs.geometry_msgs import Twist, Vector3, TwistStamped
    from lcm_msgs import std_msgs
    HAVE_LCM_MSGS = True
except ImportError as e:
    print(f"Warning: Failed to import lcm_msgs: {e}")
    print("Will try to import dynamically when needed")
    HAVE_LCM_MSGS = False

# Global variables to hold resources that need cleanup
_resources_to_cleanup = []

@contextmanager
def safe_lcm_instance():
    """Context manager for safely managing LCM instance lifecycle"""
    # Try multiple provider URLs to find one that works
    provider_urls = [
        None,  # Default
        "udpm://239.255.76.67:7667?ttl=0",  # Explicit localhost with TTL 0
        "udpm://localhost:7667",
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

# Flag to track if we've printed the transform warning yet
transform_warning_printed = False

class TwistMsg:
    def __init__(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.linear_z = linear_z
        self.angular_x = angular_x
        self.angular_y = angular_y
        self.angular_z = angular_z

class TwistHandler:
    """Handles Twist messages, transforms, and publishing"""
    
    def __init__(self, buffer, source_topic, target_topic, stamped=False, frame_id=""):
        # Ensure lcm_msgs is available
        self._ensure_lcm_msgs()
        
        # Create regular LCM instance for message passing
        self.lc = lcm.LCM()
        
        # Store transform buffer
        self.buffer = buffer
        
        self.source_topic = source_topic
        self.target_topic = target_topic
        self.stamped = stamped
        self.frame_id = frame_id
        
        # Register subscriptions with regular LCM
        if stamped:
            self.lc.subscribe(source_topic, self.handle_twist_stamped)
        else:
            self.lc.subscribe(source_topic, self.handle_twist)
        
        # Store the latest received twist message
        self.last_twist_msg = TwistMsg()
        self.received_twist = False
        self.last_message_time = None
        
        # Target frames
        self.target_frame = "world"
        self.source_frame = "link6"
        
        # For printing frame info - don't spam console
        self.last_frame_print_time = 0
        self.frames_printed = False
        
        print(f"Initialized TwistHandler: {source_topic} -> {target_topic}")
        print(f"Looking up transforms between '{self.target_frame}' and '{self.source_frame}'")
        if stamped:
            print(f"Using TwistStamped messages with frame_id: '{frame_id}'")
    
    def _ensure_lcm_msgs(self):
        """Ensure lcm_msgs module is imported"""
        global HAVE_LCM_MSGS
        if not HAVE_LCM_MSGS:
            try:
                global Twist, Vector3, TwistStamped, std_msgs
                from lcm_msgs.geometry_msgs import Twist, Vector3, TwistStamped
                from lcm_msgs import std_msgs
                HAVE_LCM_MSGS = True
                print("Successfully imported lcm_msgs dynamically")
            except ImportError as e:
                print(f"Error importing lcm_msgs: {e}")
                raise
    
    def handle_twist(self, channel, data):
        """Handle Twist messages"""
        try:
            # Ensure lcm_msgs is available
            self._ensure_lcm_msgs()
            
            twist_msg = Twist.decode(data)
            
            # Store the twist data
            self.last_twist_msg.linear_x = twist_msg.linear.x
            self.last_twist_msg.linear_y = twist_msg.linear.y
            self.last_twist_msg.linear_z = twist_msg.linear.z
            self.last_twist_msg.angular_x = twist_msg.angular.x
            self.last_twist_msg.angular_y = twist_msg.angular.y
            self.last_twist_msg.angular_z = twist_msg.angular.z
            
            self.received_twist = True
            self.last_message_time = time.time()
            print(f"Received Twist: linear=({twist_msg.linear.x:.3f}, {twist_msg.linear.y:.3f}, {twist_msg.linear.z:.3f}), "
                  f"angular=({twist_msg.angular.x:.3f}, {twist_msg.angular.y:.3f}, {twist_msg.angular.z:.3f})")
        except Exception as e:
            print(f"Error handling Twist message: {e}")
    
    def handle_twist_stamped(self, channel, data):
        """Handle TwistStamped messages"""
        try:
            # Ensure lcm_msgs is available
            self._ensure_lcm_msgs()
            
            twist_stamped_msg = TwistStamped.decode(data)
            twist_msg = twist_stamped_msg.twist
            
            # Store the twist data
            self.last_twist_msg.linear_x = twist_msg.linear.x
            self.last_twist_msg.linear_y = twist_msg.linear.y
            self.last_twist_msg.linear_z = twist_msg.linear.z
            self.last_twist_msg.angular_x = twist_msg.angular.x
            self.last_twist_msg.angular_y = twist_msg.angular.y
            self.last_twist_msg.angular_z = twist_msg.angular.z
            
            self.received_twist = True
            self.last_message_time = time.time()
            print(f"Received TwistStamped: linear=({twist_msg.linear.x:.3f}, {twist_msg.linear.y:.3f}, {twist_msg.linear.z:.3f}), "
                  f"angular=({twist_msg.angular.x:.3f}, {twist_msg.angular.y:.3f}, {twist_msg.angular.z:.3f})")
        except Exception as e:
            print(f"Error handling TwistStamped message: {e}")
    
    def process(self):
        """Process LCM messages and apply transforms"""
        global transform_warning_printed
        
        # Process any incoming LCM messages
        self.lc.handle_timeout(10)  # 10ms timeout
        
        # Check if we've received a twist message
        if not self.received_twist:
            return
        
        # Ensure lcm_msgs is available
        self._ensure_lcm_msgs()
            
        try:
            # Get current time
            now = datetime.now()
            
            # Check if we can find the transform (don't spam output)
            can_transform = self.buffer.can_transform(self.target_frame, self.source_frame, now)
            
            # Print frame info only once in a while
            current_time = time.time()
            if not can_transform and not transform_warning_printed:
                transform_warning_printed = True
                print(f"\nWARNING: Cannot transform between '{self.target_frame}' and '{self.source_frame}'")
                print("This script requires these transforms to be published by a running robot model/simulation.")
                print("Without transforms, we'll publish the twist values directly.")
                
                # Try to get available frames for debugging
                try:
                    frames = self.buffer.get_all_frame_names()
                    if frames:
                        print(f"Available frames ({len(frames)} total):")
                        for frame in sorted(frames):
                            print(f"  - {frame}")
                    else:
                        print("No frames available yet.")
                except Exception as e:
                    print(f"Error getting frame names: {e}")
                    
                print("\nFallback mode: Publishing received values directly to output topic")
            
            # Create message - either from transform or directly from twist
            if self.stamped:
                result_msg = TwistStamped()
                result_msg.header = std_msgs.Header()
                result_msg.header.frame_id = self.frame_id
                # Update timestamp
                current_time = time.time()
                result_msg.header.stamp.sec = int(current_time)
                result_msg.header.stamp.nsec = int((current_time - int(current_time)) * 1e9)
                result_msg.twist = Twist()
                result_msg.twist.linear = Vector3()
                result_msg.twist.angular = Vector3()
                result = result_msg.twist
                msg = result_msg
            else:
                result_msg = Twist()
                result_msg.linear = Vector3()
                result_msg.angular = Vector3()
                result = result_msg
                msg = result_msg
            
            if can_transform:
                # We have a transform, use it
                # Import lcm_msgs locally to ensure it's in scope
                import lcm_msgs as lcm_msgs_local
                
                transform = self.buffer.lookup_transform(
                    self.target_frame, 
                    self.source_frame, 
                    now, 
                    lcm_module=lcm_msgs_local
                )
                
                # Apply the transform's translation AND the twist's linear components
                result.linear.x = transform.transform.translation.x + self.last_twist_msg.linear_x
                result.linear.y = transform.transform.translation.y + self.last_twist_msg.linear_y
                result.linear.z = transform.transform.translation.z + self.last_twist_msg.linear_z
                
                print(f"Transform: ({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})")
                print(f"Twist: ({self.last_twist_msg.linear_x:.3f}, {self.last_twist_msg.linear_y:.3f}, {self.last_twist_msg.linear_z:.3f})")
                print(f"Result: ({result.linear.x:.3f}, {result.linear.y:.3f}, {result.linear.z:.3f})")
                
                # Extract Euler angles from the quaternion
                qw = transform.transform.rotation.w
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                
                # Simple conversion to Euler angles
                # Roll (x-axis rotation)
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll = 0.0
                if cosr_cosp != 0:
                    roll = sinr_cosp / cosr_cosp
                
                # Pitch (y-axis rotation)
                sinp = 2 * (qw * qy - qz * qx)
                pitch = 0.0
                if abs(sinp) >= 1:
                    pitch = sinp / abs(sinp) * 3.14159 / 2
                else:
                    pitch = sinp
                    
                # Yaw (z-axis rotation)
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = 0.0
                if cosy_cosp != 0:
                    yaw = siny_cosp / cosy_cosp
                
                # Apply the Euler angles AND the twist's angular components to result
                result.angular.x = roll + self.last_twist_msg.angular_x
                result.angular.y = pitch + self.last_twist_msg.angular_y
                result.angular.z = yaw + self.last_twist_msg.angular_z
                
                print(f"Rotation: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
                print(f"Angular Twist: ({self.last_twist_msg.angular_x:.3f}, {self.last_twist_msg.angular_y:.3f}, {self.last_twist_msg.angular_z:.3f})")
                print(f"Result Rotation: ({result.angular.x:.3f}, {result.angular.y:.3f}, {result.angular.z:.3f})")
                
                print(f"Final output: pos=({result.linear.x:.3f}, {result.linear.y:.3f}, {result.linear.z:.3f}), "
                      f"rot=({result.angular.x:.3f}, {result.angular.y:.3f}, {result.angular.z:.3f})")
                print("-----------------------------")
                
            else:
                # No transform available, just pass through the received twist values
                result.linear.x = self.last_twist_msg.linear_x
                result.linear.y = self.last_twist_msg.linear_y
                result.linear.z = self.last_twist_msg.linear_z
                result.angular.x = self.last_twist_msg.angular_x
                result.angular.y = self.last_twist_msg.angular_y
                result.angular.z = self.last_twist_msg.angular_z
            
            # Publish the result
            self.lc.publish(self.target_topic, msg.encode())
            
            # Only print occasionally to avoid flooding the console
            if (current_time - self.last_message_time) < 2.0:  # Within 2 seconds of last message
                print(f"Published to {self.target_topic}: {self.target_frame}->{self.source_frame}")
            
        except Exception as e:
            import traceback
            print(f"Error processing twist and transform: {e}")
            traceback.print_exc()
            
        except Exception as e:
            print(f"Error processing transform: {e}")

def main():
    global _resources_to_cleanup
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Transform Modifier for Robot Arm Control')
    parser.add_argument('--source_topic', type=str, default='cmd_pos_diff#geometry_msgs.Twist',
                        help='Source LCM topic name (default: cmd_pos_diff#geometry_msgs.Twist)')
    parser.add_argument('--target_topic', type=str, default='cmd_pos#geometry_msgs.Twist',
                        help='Target LCM topic name (default: cmd_pos#geometry_msgs.Twist)')
    parser.add_argument('--stamped', action='store_true',
                        help='Use TwistStamped messages instead of Twist')
    parser.add_argument('--frame_id', type=str, default='world',
                        help='Frame ID for TwistStamped messages (default: world)')
    
    args = parser.parse_args()
    
    source_topic = args.source_topic
    target_topic = args.target_topic
    stamped = args.stamped
    frame_id = args.frame_id
    
    # Update topic names if using stamped messages
    if stamped:
        if 'Twist' in source_topic and not 'TwistStamped' in source_topic:
            source_topic = source_topic.replace('Twist', 'TwistStamped')
        if 'Twist' in target_topic and not 'TwistStamped' in target_topic:
            target_topic = target_topic.replace('Twist', 'TwistStamped')
    
    # Set up signal handlers for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup on normal exit
    atexit.register(cleanup_resources)
    
    print("=== Transform Modifier for Robot Arm Control ===\n")
    
    # Use context manager for tf_lcm LCM instance
    with safe_lcm_instance() as tf_lcm_instance:
        # Add tf_lcm_instance to resources for cleanup
        _resources_to_cleanup.append(tf_lcm_instance)
        
        # Create transform buffer and listener
        buffer = tf_lcm_py.Buffer(30.0)
        _resources_to_cleanup.append(buffer)
        
        listener = tf_lcm_py.TransformListener(tf_lcm_instance, buffer)
        _resources_to_cleanup.append(listener)
        
        # Create a TwistHandler instance
        handler = TwistHandler(buffer, source_topic, target_topic, stamped, frame_id)
        _resources_to_cleanup.append(handler)
        
        print(f"Listening for messages on {source_topic} and publishing to {target_topic}")
        print("Press Ctrl+C to exit")
        
        # Main loop
        while True:
            try:
                # Process tf_lcm messages
                tf_lcm_instance.handle_timeout(10)  # 10ms timeout
                
                # Process twist messages and apply transforms
                handler.process()
                
                # Small sleep to prevent CPU hogging
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Error during main loop: {e}")
                time.sleep(1)  # Longer pause after an error

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Unhandled exception: {e}")
        # Make sure we clean up even if an exception occurred
        cleanup_resources()
        sys.exit(1)

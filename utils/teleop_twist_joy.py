#!/usr/bin/env python3
"""
LCM Teleop Twist Joy

This script converts joystick inputs from LCM Joy messages into velocity commands
published as LCM Twist messages. It's a replacement for the ROS teleop_twist_joy package.

Features:
- Configurable mapping of joystick axes and buttons to velocity commands
- Support for regular and turbo speed modes
- Special "b4,5" syntax to use buttons as axis inputs (where 4 is negative, 5 is positive)
- Command-line configuration of all parameters
- High-performance multi-threaded design for responsive control
"""

import argparse
import lcm
import threading
import time
import re
import sys
import signal
import select
from lcm_msgs.sensor_msgs import Joy
from lcm_msgs.geometry_msgs import Twist, Vector3


class TeleopTwistJoy:
    def __init__(self, config):
        """Initialize the teleop_twist_joy node with configuration parameters."""
        self.config = config
        self.lc = lcm.LCM()
        self.joy_subscription = None
        self.lock = threading.Lock()
        self.last_joy = None
        self.last_twist = None
        self.stopped = False
        self._warned_axes = set()  # Track which axis warnings have been printed
        
        # Thread control
        self.lcm_thread = None
        self.command_thread = None
        self.command_event = threading.Event()
        
        # Parse axis mappings with special "b4,5" button syntax
        self._parse_axis_mappings()
        
        # Subscribe to joy messages
        self.joy_channel = config['joy_channel']
        self.cmd_vel_channel = config['cmd_vel_channel']
        self.joy_subscription = self.lc.subscribe(self.joy_channel, self._joy_callback)
        
        # Print configuration
        print(f"Teleop Twist Joy (LCM) started")
        print(f"Subscribed to Joy channel: {self.joy_channel}")
        print(f"Publishing Twist on channel: {self.cmd_vel_channel}")
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _parse_axis_mappings(self):
        """Parse axis mappings, including special 'b4,5' button syntax."""
        self.axis_mappings = {}
        self.button_axis_mappings = {}
        
        axis_keys = [
            'axis_linear_x', 'axis_linear_y', 'axis_linear_z',
            'axis_angular_yaw', 'axis_angular_pitch', 'axis_angular_roll'
        ]
        
        # Print header for debug output
        print("\nAxis mappings:")
        print("  {:<20} {:<15} {:<15}".format("Axis", "Type", "Value"))
        print("  " + "-" * 50)
        
        for key in axis_keys:
            value = self.config[key]
            # Handle string values that should be converted to int
            if isinstance(value, str) and value.strip('-').isdigit():
                value = int(value)
                
            if value == -1:
                # Disabled axis
                self.axis_mappings[key] = None
                self.button_axis_mappings[key] = None
                print(f"  {key:<20} {'Disabled':<15} {value}")
            elif isinstance(value, str) and value.startswith('b'):
                # Button axis format: "b4,5" (4 for negative, 5 for positive)
                match = re.match(r'b(\d+),(\d+)', value)
                if match:
                    neg_btn = int(match.group(1))
                    pos_btn = int(match.group(2))
                    self.button_axis_mappings[key] = (neg_btn, pos_btn)
                    self.axis_mappings[key] = None
                    print(f"  {key:<20} {'Button pair':<15} neg:{neg_btn}, pos:{pos_btn}")
                else:
                    print(f"  {key:<20} {'INVALID':<15} {value} (expected format: b4,5)")
                    self.axis_mappings[key] = None
                    self.button_axis_mappings[key] = None
            else:
                # Regular axis mapping
                try:
                    axis_idx = int(value)
                    self.axis_mappings[key] = axis_idx
                    self.button_axis_mappings[key] = None
                    print(f"  {key:<20} {'Joystick axis':<15} {axis_idx}")
                except (ValueError, TypeError):
                    print(f"  {key:<20} {'INVALID':<15} {value} (must be an integer)")
                    self.axis_mappings[key] = None
                    self.button_axis_mappings[key] = None
        print("")
    
    def _signal_handler(self, sig, frame):
        """Handle Ctrl+C for graceful shutdown."""
        self.stop()
        sys.exit(0)
    
    def _joy_callback(self, channel, data):
        """Process incoming Joy messages."""
        try:
            joy_msg = Joy.decode(data)
            
            # Print detailed information about the first message
            if not hasattr(self, '_first_joy_received') and self.config.get('verbose', False):
                self._first_joy_received = True
                print("\nReceived first Joy message:")
                print(f"  Number of axes: {len(joy_msg.axes)}")
                print(f"  Number of buttons: {len(joy_msg.buttons)}")
                print("  Axes values: " + ", ".join([f"{i}:{v:.3f}" for i, v in enumerate(joy_msg.axes)]))
                print("  Button values: " + ", ".join([f"{i}:{v}" for i, v in enumerate(joy_msg.buttons)]))
            
            with self.lock:
                self.last_joy = joy_msg
                # Signal that we have new joy data to process
                self.command_event.set()
        except Exception as e:
            print(f"Error decoding joy message: {e}")
    
    def _get_axis_value(self, joy_msg, axis_key):
        """
        Get the value for an axis, handling both regular joystick axes
        and the special button-as-axis mappings.
        """
        if joy_msg is None:
            return 0.0
        
        # Check if this axis is mapped to buttons
        button_mapping = self.button_axis_mappings[axis_key]
        if button_mapping is not None:
            neg_btn, pos_btn = button_mapping
            value = 0.0
            # Check array bounds before accessing
            if neg_btn >= len(joy_msg.buttons):
                print(f"Warning: Button {neg_btn} out of range (max: {len(joy_msg.buttons)-1})")
            elif neg_btn >= 0 and joy_msg.buttons[neg_btn]:
                value -= 1.0
                
            if pos_btn >= len(joy_msg.buttons):
                print(f"Warning: Button {pos_btn} out of range (max: {len(joy_msg.buttons)-1})")
            elif pos_btn >= 0 and joy_msg.buttons[pos_btn]:
                value += 1.0
            return value
        
        # Regular axis mapping
        axis_idx = self.axis_mappings[axis_key]
        if axis_idx is None:
            return 0.0
            
        # Check array bounds before accessing
        if axis_idx < 0:
            return 0.0
            
        if axis_idx >= len(joy_msg.axes):
            # Only print warning once per axis to avoid spamming
            if not hasattr(self, '_warned_axes'):
                self._warned_axes = set()
            if axis_idx not in self._warned_axes:
                print(f"Warning: Axis {axis_idx} out of range (max: {len(joy_msg.axes)-1})")
                self._warned_axes.add(axis_idx)
            return 0.0
        
        return joy_msg.axes[axis_idx]
    
    def _is_enabled(self, joy_msg):
        """Check if movement is enabled based on enable button configuration."""
        if joy_msg is None:
            return False
            
        if not self.config['require_enable_button']:
            return True
            
        enable_button = self.config['enable_button']
        if 0 <= enable_button < len(joy_msg.buttons):
            return joy_msg.buttons[enable_button]
            
        return False
    
    def _is_turbo_enabled(self, joy_msg):
        """Check if turbo mode is enabled."""
        if joy_msg is None or self.config['enable_turbo_button'] < 0:
            return False
            
        turbo_button = self.config['enable_turbo_button']
        if 0 <= turbo_button < len(joy_msg.buttons):
            return joy_msg.buttons[turbo_button]
            
        return False
    
    def _process_joy_command(self):
        """Process joy input and create a Twist message."""
        with self.lock:
            joy = self.last_joy
            
        if joy is None:
            return None  # No joy message received yet
            
        if not self._is_enabled(joy):
            # Zero twist when disabled
            return self._create_zero_twist()
            
        # Determine if turbo mode is active
        turbo = self._is_turbo_enabled(joy)
        scales = {}
        
        # Set up scale factors based on turbo state
        for axis in ['x', 'y', 'z']:
            linear_key = f'scale_linear_{axis}'
            linear_turbo_key = f'scale_linear_turbo_{axis}'
            scales[f'linear_{axis}'] = (
                self.config[linear_turbo_key] if turbo else self.config[linear_key]
            )
            
        for axis in ['yaw', 'pitch', 'roll']:
            angular_key = f'scale_angular_{axis}'
            angular_turbo_key = f'scale_angular_turbo_{axis}'
            scales[f'angular_{axis}'] = (
                self.config[angular_turbo_key] if turbo else self.config[angular_key]
            )
        
        # Create Twist message
        twist = Twist()
        twist.linear = Vector3()
        twist.angular = Vector3()
        
        # Apply scaling to linear axes
        twist.linear.x = self._get_axis_value(joy, 'axis_linear_x') * scales['linear_x']
        twist.linear.y = self._get_axis_value(joy, 'axis_linear_y') * scales['linear_y']
        twist.linear.z = self._get_axis_value(joy, 'axis_linear_z') * scales['linear_z']
        
        # Apply scaling to angular axes
        twist.angular.x = self._get_axis_value(joy, 'axis_angular_roll') * scales['angular_roll']
        twist.angular.y = self._get_axis_value(joy, 'axis_angular_pitch') * scales['angular_pitch']
        twist.angular.z = self._get_axis_value(joy, 'axis_angular_yaw') * scales['angular_yaw']
        
        # Check for inverted reverse functionality
        if self.config['inverted_reverse'] and twist.linear.x < 0:
            twist.angular.z = -twist.angular.z
            
        # Debug output when verbose is enabled
        if self.config.get('verbose', False) and hasattr(self, '_last_debug_time'):
            current_time = time.time()
            # Limit debug output to once per second to avoid spamming
            if current_time - self._last_debug_time >= 1.0:
                print("\nCurrent twist values:")
                print(f"  Linear:  X: {twist.linear.x:.3f}  Y: {twist.linear.y:.3f}  Z: {twist.linear.z:.3f}")
                print(f"  Angular: X: {twist.angular.x:.3f}  Y: {twist.angular.y:.3f}  Z: {twist.angular.z:.3f}")
                if turbo:
                    print("  [TURBO MODE ACTIVE]")
                self._last_debug_time = current_time
        elif not hasattr(self, '_last_debug_time'):
            self._last_debug_time = time.time()
            
        return twist
        
    def _command_loop(self):
        """Thread for processing joystick commands and publishing Twist messages."""
        rate = self.config['publish_rate']
        period = 1.0 / rate
        last_publish_time = 0
        message_counter = 0
        start_time = time.time()
        
        while not self.stopped:
            # Wait for a new joy message or timeout
            timeout = max(0, (last_publish_time + period) - time.time())
            new_data = self.command_event.wait(timeout)
            self.command_event.clear()
            
            # Process the current joy state
            twist = self._process_joy_command()
            
            # Determine if we need to publish
            current_time = time.time()
            time_since_last = current_time - last_publish_time
            should_publish = (time_since_last >= period) or (new_data and not self._is_same_twist(twist))
            
            # Publish if needed
            if should_publish and twist is not None:
                with self.lock:
                    self.last_twist = twist
                self.lc.publish(self.cmd_vel_channel, twist.encode())
                last_publish_time = current_time
                message_counter += 1
                
                # Print statistics every 5 seconds if verbose
                if self.config.get('verbose', False) and (current_time - start_time) >= 5.0:
                    elapsed = current_time - start_time
                    rate = message_counter / elapsed if elapsed > 0 else 0
                    print(f"\nPublishing statistics:")
                    print(f"  Messages sent: {message_counter}")
                    print(f"  Average rate: {rate:.2f} Hz")
                    message_counter = 0
                    start_time = current_time
            
    def _is_same_twist(self, new_twist):
        """Check if the new twist is the same as the last published one."""
        if new_twist is None or self.last_twist is None:
            return False
            
        # Compare linear components
        if (abs(new_twist.linear.x - self.last_twist.linear.x) > 0.001 or
            abs(new_twist.linear.y - self.last_twist.linear.y) > 0.001 or
            abs(new_twist.linear.z - self.last_twist.linear.z) > 0.001):
            return False
            
        # Compare angular components
        if (abs(new_twist.angular.x - self.last_twist.angular.x) > 0.001 or
            abs(new_twist.angular.y - self.last_twist.angular.y) > 0.001 or
            abs(new_twist.angular.z - self.last_twist.angular.z) > 0.001):
            return False
            
        return True
    
    def _lcm_handler_loop(self):
        """Thread for handling incoming LCM messages."""
        while not self.stopped:
            # Get the LCM file descriptor
            lcm_fd = self.lc.fileno()
            # Wait for the LCM file descriptor to be readable
            readable, _, _ = select.select([lcm_fd], [], [], 0.1)
            if lcm_fd in readable:
                # Handle available LCM messages
                self.lc.handle()
    
    def _create_zero_twist(self):
        """Create a zero velocity command."""
        twist = Twist()
        twist.linear = Vector3()
        twist.angular = Vector3()
        return twist
        
    def _publish_zero_twist(self):
        """Publish a zero velocity command."""
        twist = self._create_zero_twist()
        self.lc.publish(self.cmd_vel_channel, twist.encode())
    
    def run(self):
        """Start the multi-threaded control system."""
        self.stopped = False
        
        # Start LCM handler thread
        self.lcm_thread = threading.Thread(target=self._lcm_handler_loop)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
        
        # Start command processing thread
        self.command_thread = threading.Thread(target=self._command_loop)
        self.command_thread.daemon = True
        self.command_thread.start()
        
        print("Publishing velocity commands. Press Ctrl+C to exit.")
        
        # Wait for threads to exit
        try:
            while not self.stopped:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        """Stop the teleop node and clean up."""
        self.stopped = True
        
        # Wait for threads to finish
        if self.lcm_thread and self.lcm_thread.is_alive():
            self.lcm_thread.join(1.0)
            
        if self.command_thread and self.command_thread.is_alive():
            self.command_event.set()  # Wake up the command thread
            self.command_thread.join(1.0)
        
        # Unsubscribe from joy messages
        if self.joy_subscription:
            self.lc.unsubscribe(self.joy_subscription)
            self.joy_subscription = None
        
        # Publish a zero twist before exiting
        self._publish_zero_twist()
        print("\nTeleop Twist Joy stopped")


def str2bool(v):
    """Convert string representation to boolean value properly for argparse."""
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    """Parse command-line arguments and start the teleop node."""
    parser = argparse.ArgumentParser(description='LCM Teleop Twist Joy')
    
    # Channels
    parser.add_argument('--joy-channel', type=str, default='joy#sensor_msgs.Joy',
                        help='LCM channel for Joy messages')
    parser.add_argument('--cmd-vel-channel', type=str, default='cmd_vel#geometry_msgs.Twist',
                        help='LCM channel for Twist messages')
    
    # Enable buttons
    parser.add_argument('--require-enable-button', type=str2bool, default=True,
                        help='Whether to require the enable button')
    parser.add_argument('--enable-button', type=int, default=0,
                        help='Joystick button to enable movement')
    parser.add_argument('--enable-turbo-button', type=int, default=1,
                        help='Joystick button for turbo mode (disabled with -1)')
    
    # Linear axes
    parser.add_argument('--axis-linear-x', type=str, default='0',
                        help='Joystick axis/buttons for linear X (forward/back)')
    parser.add_argument('--axis-linear-y', type=str, default='1',
                        help='Joystick axis/buttons for linear Y (left/right)')
    parser.add_argument('--axis-linear-z', type=str, default='7',
                        help='Joystick axis/buttons for linear Z (up/down)')
    
    # Linear scales
    parser.add_argument('--scale-linear-x', type=float, default=0.5,
                        help='Scale for linear X movement')
    parser.add_argument('--scale-linear-y', type=float, default=0.5,
                        help='Scale for linear Y movement')
    parser.add_argument('--scale-linear-z', type=float, default=0.5,
                        help='Scale for linear Z movement')
    
    # Linear turbo scales
    parser.add_argument('--scale-linear-turbo-x', type=float, default=1.0,
                        help='Turbo scale for linear X movement')
    parser.add_argument('--scale-linear-turbo-y', type=float, default=1.0,
                        help='Turbo scale for linear Y movement')
    parser.add_argument('--scale-linear-turbo-z', type=float, default=1.0,
                        help='Turbo scale for linear Z movement')
    
    # Angular axes
    parser.add_argument('--axis-angular-yaw', type=str, default='6',
                        help='Joystick axis/buttons for angular yaw (turning)')
    parser.add_argument('--axis-angular-pitch', type=str, default='3',
                        help='Joystick axis/buttons for angular pitch')
    parser.add_argument('--axis-angular-roll', type=str, default='2',
                        help='Joystick axis/buttons for angular roll')
    
    # Angular scales
    parser.add_argument('--scale-angular-yaw', type=float, default=0.5,
                        help='Scale for angular yaw movement')
    parser.add_argument('--scale-angular-pitch', type=float, default=0.5,
                        help='Scale for angular pitch movement')
    parser.add_argument('--scale-angular-roll', type=float, default=0.5,
                        help='Scale for angular roll movement')
    
    # Angular turbo scales
    parser.add_argument('--scale-angular-turbo-yaw', type=float, default=1.0,
                        help='Turbo scale for angular yaw movement')
    parser.add_argument('--scale-angular-turbo-pitch', type=float, default=1.0,
                        help='Turbo scale for angular pitch movement')
    parser.add_argument('--scale-angular-turbo-roll', type=float, default=1.0,
                        help='Turbo scale for angular roll movement')
    
    # Other parameters
    parser.add_argument('--inverted-reverse', type=str2bool, default=False,
                        help='Invert turning when reversing')
    parser.add_argument('--publish-rate', type=float, default=20.0,
                        help='Rate at which to publish velocity commands (Hz)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose debug output')
    
    args = parser.parse_args()
    config = vars(args)  # Convert args to dictionary
    
    teleop = TeleopTwistJoy(config)
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()


if __name__ == "__main__":
    main()

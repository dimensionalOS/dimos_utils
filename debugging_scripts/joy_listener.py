#!/usr/bin/env python3
"""
Joy message listener for LCM
This script subscribes to Joy messages and displays controller input data
for debugging purposes.
"""

import lcm
import time
import argparse
from lcm_msgs.sensor_msgs import Joy

def format_axis_value(value):
    """Format axis value with a visual indicator."""
    bar_length = 20
    middle = bar_length // 2
    position = int(middle + value * middle)
    
    bar = ['|'] * bar_length
    bar[middle] = '0'
    if position >= 0 and position < bar_length:
        bar[position] = '#'
    
    return f"{value:6.3f} [{''.join(bar)}]"

class JoyListener:
    def __init__(self, channel="joy#sensor_msgs.Joy", verbose=False):
        self.lc = lcm.LCM()
        self.channel = channel
        self.verbose = verbose
        self.last_message_time = 0
        self.message_count = 0
        self.start_time = time.time()
        
        # Subscribe to the joy channel
        self.lc.subscribe(self.channel, self.joy_handler)
        print(f"Listening for Joy messages on channel: {self.channel}")
        print("Press Ctrl+C to exit")
        
        # Store axis and button names for better output
        self.axis_names = [
            "Left/Right Stick Left",
            "Up/Down Stick Left",
            "Left/Right Stick Right",
            "Up/Down Stick Right",
            "RT",
            "LT",
            "D-pad Left/Right",
            "D-pad Up/Down"
        ]
        
        self.button_names = [
            "A",
            "B",
            "X",
            "Y",
            "LB",
            "RB",
            "Back",
            "Start",
            "Power",
            "Button Stick Left",
            "Button Stick Right"
        ]
    
    def joy_handler(self, channel, data):
        """Handle incoming Joy messages."""
        try:
            msg = Joy.decode(data)
            current_time = time.time()
            self.message_count += 1
            
            # Calculate message rate
            elapsed = current_time - self.start_time
            rate = self.message_count / elapsed if elapsed > 0 else 0
            
            # Calculate time since last message
            if self.last_message_time > 0:
                delta_t = current_time - self.last_message_time
            else:
                delta_t = 0
            self.last_message_time = current_time
            
            # Clear screen for better display
            print("\033c", end="")
            
            # Print header
            print("=" * 80)
            print(f"Joy Message Received on {channel}")
            print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nsec // 1000000:03d}")
            print(f"Message rate: {rate:.2f} Hz, Time since last message: {delta_t*1000:.1f} ms")
            print("=" * 80)
            
            # Print axes
            print("\nAXES:")
            for i, value in enumerate(msg.axes):
                if i < len(self.axis_names):
                    name = self.axis_names[i]
                else:
                    name = f"Axis {i}"
                print(f"  {name:20}: {format_axis_value(value)}")
            
            # Print buttons
            print("\nBUTTONS:")
            for i, value in enumerate(msg.buttons):
                if i < len(self.button_names):
                    name = self.button_names[i]
                else:
                    name = f"Button {i}"
                status = "PRESSED" if value else "released"
                print(f"  {name:20}: {status}")
            
            # Print raw data if verbose
            if self.verbose:
                print("\nRAW DATA:")
                print(f"  Axes ({len(msg.axes)}): {msg.axes}")
                print(f"  Buttons ({len(msg.buttons)}): {msg.buttons}")
            
            print("\n" + "=" * 80)
            print("Press Ctrl+C to exit")
            
        except Exception as e:
            print(f"Error decoding Joy message: {e}")
    
    def run(self):
        """Main loop to handle LCM messages."""
        try:
            while True:
                self.lc.handle()
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            self.lc.unsubscribe(self.channel)
            print(f"Unsubscribed from channel: {self.channel}")

def main():
    parser = argparse.ArgumentParser(description='LCM Joy message listener')
    parser.add_argument('--channel', type=str, default="joy#sensor_msgs.Joy",
                        help='LCM channel to listen on (default: joy#sensor_msgs.Joy)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show verbose output including raw data')
    args = parser.parse_args()
    
    listener = JoyListener(channel=args.channel, verbose=args.verbose)
    listener.run()

if __name__ == "__main__":
    main()

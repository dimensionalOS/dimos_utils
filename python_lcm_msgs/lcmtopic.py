#!/usr/bin/env python3

import argparse
import importlib
import lcm
import signal
import sys
import time
import threading
from collections import deque
import inspect
import importlib.util
import pkgutil
import re
from datetime import datetime

def format_bytes(size):
    """Format bytes to human-readable format"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024.0:
            return f"{size:.2f} {unit}"
        size /= 1024.0
    return f"{size:.2f} TB"

class LcmTopicTool:
    def __init__(self):
        self.lc = lcm.LCM()
        self.running = True
        self.lcm_thread = None
        self.subscriptions = []
        
    def extract_message_type(self, topic_name):
        """Extract the message type from a topic name (assuming format topic_name#message_type)"""
        if "#" in topic_name:
            _, message_type = topic_name.split("#", 1)
            return message_type
        return None
    
    def import_message_class(self, message_type):
        """Dynamically import an LCM message class based on message type"""
        if not message_type:
            return None
            
        # Parse message type (e.g., sensor_msgs.JointState)
        if "." in message_type:
            module_path, class_name = message_type.rsplit(".", 1)
            try:
                # Import from lcm_msgs
                module = importlib.import_module(f"lcm_msgs.{module_path}")
                return getattr(module, class_name)
            except (ImportError, AttributeError) as e:
                print(f"Error importing message type {message_type}: {e}")
                return None
        return None
        
    def start_lcm_thread(self):
        """Start the LCM handling thread"""
        self.lcm_thread = threading.Thread(target=self._lcm_thread)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
        
    def _lcm_thread(self):
        """LCM handling thread function"""
        while self.running:
            try:
                self.lc.handle_timeout(100)  # 100ms timeout
            except Exception as e:
                print(f"LCM error: {e}")
                
    def stop(self):
        """Stop the LCM thread"""
        self.running = False
        if self.lcm_thread:
            self.lcm_thread.join(timeout=1.0)
            
    def list_topics(self, args):
        """List all active LCM channels"""
        print("Waiting for LCM messages to determine active topics...")
        print("(This may take a few seconds - please ensure publishers are active)")
        print("Press Ctrl+C to stop")
        
        # Create a wildcard subscription to capture all messages
        active_topics = set()
        
        def capture_handler(channel, data):
            active_topics.add(channel)
            
        # Subscribe to all channels
        subscription = self.lc.subscribe(".*", capture_handler)
        self.start_lcm_thread()
        
        try:
            # Wait for a specified time to collect topics
            wait_time = args.timeout if args else 5
            time.sleep(wait_time)
                
            # Print the active topics
            if active_topics:
                print("\nActive LCM topics:")
                for topic in sorted(active_topics):
                    print(f"  {topic}")
                print(f"\nFound {len(active_topics)} active topics.")
            else:
                print("\nNo active LCM topics detected.")
                
        except KeyboardInterrupt:
            print("\nStopped topic discovery.")
        finally:
            self.lc.unsubscribe(subscription)
            self.stop()
            
    def echo_topic(self, args):
        """Print messages from a topic"""
        topic = args.topic
        max_depth = args.max_depth
        
        # Extract message type from topic
        message_type = self.extract_message_type(topic)
        message_class = self.import_message_class(message_type)
        
        if not message_class:
            print(f"Could not determine message type for {topic}")
            print(f"Make sure topic follows format 'topic_name#message_type'")
            return
            
        print(f"Subscribing to topic: {topic}")
        print(f"Message type: {message_type}")
        print("Press Ctrl+C to stop")
        
        # Counter for messages
        msg_count = 0
        
        # Helper function to recursively print message content
        def print_value(value, name, indent=0, depth=0, visited=None):
            if visited is None:
                visited = set()
                
            # Handle max depth
            if max_depth > 0 and depth > max_depth:
                print(f"{' ' * indent}{name}: [max depth reached]")
                return
                
            # Handle different types
            if value is None:
                print(f"{' ' * indent}{name}: None")
            elif isinstance(value, (int, float, bool, str)):
                print(f"{' ' * indent}{name}: {value}")
            elif isinstance(value, (list, tuple)):
                if len(value) == 0:
                    print(f"{' ' * indent}{name}: []")
                else:
                    print(f"{' ' * indent}{name}: [")
                    # Check if elements are simple or complex
                    if all(isinstance(x, (int, float, bool, str, type(None))) for x in value):
                        # Simple list - print on a single line
                        items_str = ', '.join(str(x) for x in value)
                        print(f"{' ' * (indent+2)}{items_str}")
                    else:
                        # Complex list - print each item recursively
                        for i, item in enumerate(value):
                            print_value(item, f"[{i}]", indent+2, depth+1, visited)
                    print(f"{' ' * indent}]")
            elif id(value) in visited:
                print(f"{' ' * indent}{name}: [circular reference]")
            else:
                # Handle objects
                obj_id = id(value)
                visited.add(obj_id)
                
                # Get the type name to display
                type_name = type(value).__module__ + '.' + type(value).__name__
                print(f"{' ' * indent}{name}: {type_name} {{")
                
                # Print object attributes
                if hasattr(value, "__slots__"):
                    # For ROS-style messages with slots
                    for slot in value.__slots__:
                        if hasattr(value, slot):
                            slot_value = getattr(value, slot)
                            print_value(slot_value, slot, indent+2, depth+1, visited)
                else:
                    # For regular objects
                    attrs = [a for a in dir(value) if not a.startswith('_') and not callable(getattr(value, a))]
                    for attr in attrs:
                        attr_value = getattr(value, attr)
                        print_value(attr_value, attr, indent+2, depth+1, visited)
                print(f"{' ' * indent}}}")
        
        # Define the message handler
        def message_handler(channel, data):
            nonlocal msg_count
            try:
                msg = message_class.decode(data)
                msg_count += 1
                print(f"\n--- Message {msg_count} at {datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---")
                
                # Recursively print the message contents
                if hasattr(msg, "__slots__"):
                    # If message uses slots (ROS-style)
                    for slot in msg.__slots__:
                        if hasattr(msg, slot):
                            value = getattr(msg, slot)
                            print_value(value, slot)
                else:
                    # Otherwise print all attributes
                    attrs = [a for a in dir(msg) if not a.startswith('_') and not callable(getattr(msg, a))]
                    for attr in attrs:
                        value = getattr(msg, attr)
                        print_value(value, attr)
            except Exception as e:
                print(f"Error decoding message: {e}")
                
        # Subscribe to the topic
        subscription = self.lc.subscribe(topic, message_handler)
        self.start_lcm_thread()
        
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped echo.")
        finally:
            self.lc.unsubscribe(subscription)
            self.stop()
            
    def info_topic(self, args):
        """Show information about a topic"""
        topic = args.topic
        
        # Extract message type from topic
        message_type = self.extract_message_type(topic)
        
        if not message_type:
            print(f"Could not determine message type for {topic}")
            print(f"Make sure topic follows format 'topic_name#message_type'")
            return
            
        message_class = self.import_message_class(message_type)
        
        if not message_class:
            print(f"Message type: {message_type}")
            print("Could not import message class")
            return
            
        print(f"Topic: {topic}")
        print(f"Type: {message_type}")
        
        # Count publishers by listening for messages
        print("\nMonitoring for publishers...")
        print("Press Ctrl+C to stop monitoring.")
        
        msg_count = 0
        last_time = time.time()
        publishers_detected = False
        
        def monitor_handler(channel, data):
            nonlocal msg_count, last_time, publishers_detected
            msg_count += 1
            last_time = time.time()
            publishers_detected = True
            
        # Subscribe to the topic
        subscription = self.lc.subscribe(topic, monitor_handler)
        self.start_lcm_thread()
        
        try:
            # Wait for up to 5 seconds to detect publishers
            timeout = time.time() + 5.0
            while time.time() < timeout and not publishers_detected:
                time.sleep(0.1)
                
            if publishers_detected:
                print("Publisher detected: Yes")
                
                # Continue listening for a short time to estimate more accurately
                time.sleep(2.0)
                print(f"Received {msg_count} messages in monitoring period")
            else:
                print("Publisher detected: No")
                
        except KeyboardInterrupt:
            print("\nStopped monitoring.")
        finally:
            self.lc.unsubscribe(subscription)
            self.stop()
            
    def hz_topic(self, args):
        """Measure publish rate of a topic"""
        topic = args.topic
        window_size = args.window
        
        # Extract message type from topic
        message_type = self.extract_message_type(topic)
        message_class = self.import_message_class(message_type)
        
        if not message_class:
            print(f"Could not determine message type for {topic}")
            print(f"Make sure topic follows format 'topic_name#message_type'")
            return
            
        print(f"Subscribed to [{topic}]")
        print(f"Message type: {message_type}")
        print(f"Press Ctrl+C to stop")
        
        # Timestamp queue for frequency calculation
        timestamps = deque(maxlen=window_size)
        
        def message_handler(channel, data):
            timestamps.append(time.time())
            
        # Subscribe to the topic
        subscription = self.lc.subscribe(topic, message_handler)
        self.start_lcm_thread()
        
        try:
            while True:
                # Sleep for a short time
                time.sleep(1.0)
                
                # Calculate frequency if we have enough data
                if len(timestamps) > 1:
                    # Calculate time differences
                    diffs = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
                    
                    # Calculate statistics
                    if diffs:
                        average_diff = sum(diffs) / len(diffs)
                        rate = 1.0 / average_diff if average_diff > 0 else 0
                        min_diff = min(diffs) if diffs else 0
                        max_diff = max(diffs) if diffs else 0
                        
                        # Calculate standard deviation
                        if len(diffs) > 1:
                            variance = sum((d - average_diff) ** 2 for d in diffs) / len(diffs)
                            std_dev = variance ** 0.5
                        else:
                            std_dev = 0
                            
                        print(f"average rate: {rate:.3f}")
                        print(f"    min: {min_diff:.3f}s max: {max_diff:.3f}s std dev: {std_dev:.5f}s window: {len(timestamps)}")
                else:
                    print("waiting for messages...")
                    
        except KeyboardInterrupt:
            print("\nStopped rate measurement.")
        finally:
            self.lc.unsubscribe(subscription)
            self.stop()
            
    def bw_topic(self, args):
        """Measure bandwidth of a topic"""
        topic = args.topic
        window_size = args.window
        
        # Extract message type from topic
        message_type = self.extract_message_type(topic)
        message_class = self.import_message_class(message_type)
        
        if not message_class:
            print(f"Could not determine message type for {topic}")
            print(f"Make sure topic follows format 'topic_name#message_type'")
            return
            
        print(f"Subscribed to [{topic}]")
        print(f"Message type: {message_type}")
        print(f"Press Ctrl+C to stop")
        
        # Store data for bandwidth calculation
        message_sizes = deque(maxlen=window_size)
        timestamps = deque(maxlen=window_size)
        total_bytes = 0
        msg_count = 0
        
        def message_handler(channel, data):
            nonlocal total_bytes, msg_count
            size = len(data)
            total_bytes += size
            msg_count += 1
            message_sizes.append(size)
            timestamps.append(time.time())
            
        # Subscribe to the topic
        subscription = self.lc.subscribe(topic, message_handler)
        self.start_lcm_thread()
        
        try:
            start_time = time.time()
            while True:
                # Sleep for a short time
                time.sleep(1.0)
                
                # Calculate bandwidth if we have received messages
                if message_sizes:
                    current_time = time.time()
                    duration = current_time - start_time
                    
                    if duration > 0:
                        bytes_per_sec = total_bytes / duration
                        
                        # Calculate average message size
                        avg_size = sum(message_sizes) / len(message_sizes)
                        min_size = min(message_sizes)
                        max_size = max(message_sizes)
                        
                        print(f"{format_bytes(bytes_per_sec)}/s from {msg_count} messages")
                        print(f"    Message size mean: {format_bytes(avg_size)} min: {format_bytes(min_size)} max: {format_bytes(max_size)}")
                else:
                    print("waiting for messages...")
                    
        except KeyboardInterrupt:
            print("\nStopped bandwidth measurement.")
        finally:
            self.lc.unsubscribe(subscription)
            self.stop()

def main():
    # Create the top-level parser
    parser = argparse.ArgumentParser(
        description='LCM topic command-line tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  lcmtopic.py list                      # List all active LCM topics
  lcmtopic.py echo joint_states#sensor_msgs.JointState    # Display messages on joint_states topic
  lcmtopic.py info joint_states#sensor_msgs.JointState    # Show information about joint_states topic
  lcmtopic.py hz joint_states#sensor_msgs.JointState      # Show publishing rate of joint_states topic
  lcmtopic.py bw joint_states#sensor_msgs.JointState      # Show bandwidth of joint_states topic
        """
    )
    
    # Create subparsers for each command
    subparsers = parser.add_subparsers(dest='command', help='LCM topic commands')
    
    # Parser for 'list' command
    list_parser = subparsers.add_parser('list', help='List all active LCM topics')
    list_parser.add_argument('--timeout', type=int, default=5, help='Time to wait for topics (seconds)')
    
    # Parser for 'echo' command
    echo_parser = subparsers.add_parser('echo', help='Display messages from a topic')
    echo_parser.add_argument('topic', help='Topic name (in format topic_name#message_type)')
    echo_parser.add_argument('--max_depth', type=int, default=0, help='Maximum depth for recursive printing (0 = unlimited)')
    
    # Parser for 'info' and 'type' commands
    info_parser = subparsers.add_parser('info', help='Show information about a topic')
    info_parser.add_argument('topic', help='Topic name (in format topic_name#message_type)')
    
    type_parser = subparsers.add_parser('type', help='Show message type of a topic')
    type_parser.add_argument('topic', help='Topic name (in format topic_name#message_type)')
    
    # Parser for 'hz' command
    hz_parser = subparsers.add_parser('hz', help='Show publishing rate of a topic')
    hz_parser.add_argument('topic', help='Topic name (in format topic_name#message_type)')
    hz_parser.add_argument('--window', type=int, default=50, help='Number of messages to use for rate calculation')
    
    # Parser for 'bw' command
    bw_parser = subparsers.add_parser('bw', help='Show bandwidth usage of a topic')
    bw_parser.add_argument('topic', help='Topic name (in format topic_name#message_type)')
    bw_parser.add_argument('--window', type=int, default=100, help='Number of messages to use for bandwidth calculation')
    
    # Parse arguments
    args = parser.parse_args()
    
    # Create the tool instance
    tool = LcmTopicTool()
    
    # Handle the requested command
    try:
        if args.command == 'list':
            tool.list_topics(args)
        elif args.command == 'echo':
            tool.echo_topic(args)
        elif args.command in ['info', 'type']:
            tool.info_topic(args)
        elif args.command == 'hz':
            tool.hz_topic(args)
        elif args.command == 'bw':
            tool.bw_topic(args)
        else:
            parser.print_help()
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
    finally:
        tool.stop()

if __name__ == "__main__":
    main()

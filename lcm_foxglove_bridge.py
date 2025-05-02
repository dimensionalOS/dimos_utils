#!/usr/bin/env python3

import asyncio
import json
import time
import sys
import os
import re
import threading
import lcm
import base64
import importlib
from typing import Dict, List, Any, Set, Optional
from dataclasses import dataclass

# Import foxglove-websocket
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

# Constants
ROS_MSGS_DIR = "ros_msgs"
LCM_PYTHON_MODULES_PATH = "python_lcm_msgs/lcm_msgs"

# Mapping of ROS primitive types to JSON schema types
TYPE_MAPPING = {
    "bool": {"type": "boolean"},
    "int8": {"type": "integer", "minimum": -128, "maximum": 127},
    "uint8": {"type": "integer", "minimum": 0, "maximum": 255},
    "int16": {"type": "integer", "minimum": -32768, "maximum": 32767},
    "uint16": {"type": "integer", "minimum": 0, "maximum": 65535},
    "int32": {"type": "integer", "minimum": -2147483648, "maximum": 2147483647},
    "uint32": {"type": "integer", "minimum": 0, "maximum": 4294967295},
    "int64": {"type": "integer"},
    "uint64": {"type": "integer", "minimum": 0},
    "float32": {"type": "number"},
    "float64": {"type": "number"},
    "string": {"type": "string"},
    "char": {"type": "integer", "minimum": 0, "maximum": 255},
    "byte": {"type": "integer", "minimum": 0, "maximum": 255},
    "time": {
        "type": "object",
        "properties": {
            "sec": {"type": "integer"},
            "nsec": {"type": "integer"}
        },
        "required": ["sec", "nsec"]
    },
    "duration": {
        "type": "object",
        "properties": {
            "sec": {"type": "integer"},
            "nsec": {"type": "integer"}
        },
        "required": ["sec", "nsec"]
    },
}

@dataclass
class TopicInfo:
    """Information about an LCM topic with schema"""
    name: str  # Base topic name (without schema)
    schema_type: str  # Schema type (e.g., "sensor_msgs.Image")
    schema: dict  # JSON schema
    channel_id: Optional[ChannelId] = None  # Foxglove channel ID
    lcm_class: Any = None  # LCM message class
    package: str = ""  # ROS package name
    msg_type: str = ""  # ROS message type

class SchemaGenerator:
    """Generates JSON schemas from ROS message definitions"""
    def __init__(self):
        self.schema_cache = {}
        
    def generate_schema(self, schema_type):
        """Generate a JSON schema for the given schema type (e.g., 'sensor_msgs.Image')"""
        # Check if schema is already cached
        if schema_type in self.schema_cache:
            return self.schema_cache[schema_type]
        
        # Parse schema type to get package and message type
        if "." not in schema_type:
            raise ValueError(f"Invalid schema type format: {schema_type}")
        
        package, msg_type = schema_type.split(".", 1)
        
        # Find the .msg file
        msg_file_path = os.path.join(ROS_MSGS_DIR, package, "msg", f"{msg_type}.msg")
        if not os.path.exists(msg_file_path):
            raise FileNotFoundError(f"Message file not found: {msg_file_path}")
        
        # Parse the .msg file and generate schema
        schema = self._parse_msg_file(msg_file_path, package)
        self.schema_cache[schema_type] = schema
        return schema
    
    def _parse_msg_file(self, msg_file_path, package_name):
        """Parse a ROS .msg file and create a JSON schema"""
        with open(msg_file_path, 'r') as f:
            msg_content = f.read()
        
        # Create basic schema structure
        schema = {
            "type": "object",
            "properties": {},
            "required": []
        }
        
        # Parse each line in the .msg file
        for line in msg_content.splitlines():
            # Remove comments (anything after #)
            if '#' in line:
                line = line.split('#', 1)[0]
                
            line = line.strip()
            if not line:
                continue
            
            # Parse field definition (type field_name)
            if ' ' not in line:
                continue
            
            parts = line.split(None, 1)  # Split on any whitespace
            if len(parts) < 2:
                continue
                
            field_type, field_name = parts
            field_name = field_name.strip()  # Ensure no trailing whitespace
            
            # Check if it's an array type
            is_array = False
            array_size = None
            if field_type.endswith('[]'):
                is_array = True
                field_type = field_type[:-2]
            elif '[' in field_type and field_type.endswith(']'):
                match = re.match(r'(.*)\[(\d+)\]', field_type)
                if match:
                    field_type = match.group(1)
                    array_size = int(match.group(2))
                    is_array = True
            
            # Process the field and add to schema
            field_schema = self._convert_type_to_schema(field_type, package_name, is_array, array_size)
            if field_schema:
                schema["properties"][field_name] = field_schema
                schema["required"].append(field_name)
        
        return schema
    
    def _convert_type_to_schema(self, field_type, package_name, is_array=False, array_size=None):
        """Convert a ROS field type to a JSON schema type"""
        # Check for primitive types
        if field_type in TYPE_MAPPING:
            field_schema = dict(TYPE_MAPPING[field_type])
            if is_array:
                schema = {"type": "array", "items": field_schema}
                if array_size is not None:
                    schema["maxItems"] = array_size
                    schema["minItems"] = array_size
                return schema
            return field_schema
            
        # Special case for Header
        elif field_type == "Header" or field_type == "std_msgs/Header":
            header_schema = {
                "type": "object",
                "properties": {
                    "seq": {"type": "integer"},
                    "stamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nsec": {"type": "integer"}
                        },
                        "required": ["sec", "nsec"]
                    },
                    "frame_id": {"type": "string"}
                },
                "required": ["seq", "stamp", "frame_id"]
            }
            
            if is_array:
                schema = {"type": "array", "items": header_schema}
                if array_size is not None:
                    schema["maxItems"] = array_size
                    schema["minItems"] = array_size
                return schema
            return header_schema
            
        # Complex type - could be from another package
        else:
            # Check if type contains a package name
            if "/" in field_type:
                pkg, msg = field_type.split("/", 1)
                complex_schema_type = f"{pkg}.{msg}"
            else:
                # Assume it's from the same package
                complex_schema_type = f"{package_name}.{field_type}"
            
            try:
                # Try to recursively generate schema
                complex_schema = self.generate_schema(complex_schema_type)
                
                if is_array:
                    schema = {"type": "array", "items": complex_schema}
                    if array_size is not None:
                        schema["maxItems"] = array_size
                        schema["minItems"] = array_size
                    return schema
                return complex_schema
            except Exception as e:
                print(f"Error processing complex type {field_type}: {e}")
                # Return a placeholder schema
                return {"type": "object", "description": f"Error: could not process type {field_type}"}

class LcmTopicDiscoverer:
    """Discovers LCM topics and their schemas"""
    def __init__(self, callback):
        """
        Initialize the topic discoverer
        
        Args:
            callback: Function to call when a new topic is discovered
        """
        self.lc = lcm.LCM()
        self.callback = callback
        self.topics = set()
        self.running = True
        self.thread = threading.Thread(target=self._discovery_thread)
        self.mutex = threading.Lock()
        
    def start(self):
        """Start the discovery thread"""
        self.thread.start()
        
    def stop(self):
        """Stop the discovery thread"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
            
    def _discovery_thread(self):
        """Thread function for discovering topics"""
        # Unfortunately LCM doesn't have built-in topic discovery
        # We'll use a special handler to catch all messages and extract topic info
        
        # Subscribe to all messages with a wildcard
        self.lc.subscribe(".*", self._on_any_message)
        
        while self.running:
            try:
                # Handle LCM messages with a timeout
                self.lc.handle_timeout(100)  # 100ms timeout
            except Exception as e:
                print(f"Error in LCM discovery: {e}")
    
    def _on_any_message(self, channel, data):
        """Callback for any LCM message during discovery"""
        with self.mutex:
            if channel not in self.topics:
                # New topic found
                self.topics.add(channel)
                
                # Check if the topic has schema information
                if "#" in channel:
                    # Call the callback with the discovered topic
                    try:
                        self.callback(channel)
                    except Exception as e:
                        print(f"Error processing discovered topic {channel}: {e}")

class LcmFoxgloveBridge:
    """Main bridge class that connects LCM to Foxglove WebSocket server"""
    def __init__(self):
        self.lc = lcm.LCM()
        self.topics = {}  # Map of topic names to topic info
        self.schema_generator = SchemaGenerator()
        self.message_handlers = {}  # Map of topic names to message handlers
        self.discoverer = None
        self.server = None
        
    async def start(self, host="0.0.0.0", port=8765):
        """Start the bridge"""
        print(f"Starting LCM-Foxglove bridge on {host}:{port}")
        
        # Start a Foxglove WebSocket server
        self.server = FoxgloveServer(
            host=host,
            port=port,
            name="LCM-Foxglove Bridge",
            capabilities=["clientPublish"],
            supported_encodings=["json"],
        )
        
        # The server is context manager, so we don't need to explicitly start it
        # (it should be used with "async with" but we're managing it ourselves)
        print(f"WebSocket server started on {host}:{port}")
        
        # Set up topic discovery
        self.discoverer = LcmTopicDiscoverer(self._on_topic_discovered)
        self.discoverer.start()
        
        # Start LCM handling thread
        self.lcm_thread = threading.Thread(target=self._lcm_thread_func)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
        
        print("Waiting for LCM topics...")
    
    async def stop(self):
        """Stop the bridge"""
        print("Stopping LCM-Foxglove bridge")
        if self.discoverer:
            self.discoverer.stop()
        if self.server:
            await self.server.stop()
    
    def _lcm_thread_func(self):
        """Thread for handling LCM messages"""
        while True:
            try:
                self.lc.handle()
            except Exception as e:
                print(f"Error handling LCM message: {e}")
    
    def _on_topic_discovered(self, topic_name):
        """Called when a new topic is discovered"""
        # Skip if we've already processed this topic
        if topic_name in self.topics:
            return
            
        try:
            print(f"Discovered topic: {topic_name}")
            
            # Extract base topic name and schema type
            base_topic, schema_type = topic_name.split("#", 1)
            package, msg_type = schema_type.split(".", 1)
            
            # Generate schema from ROS message definition
            print(f"  Generating schema for {schema_type}...")
            schema = self.schema_generator.generate_schema(schema_type)
            
            # Try to import the LCM message class
            try:
                module_name = f"lcm_msgs.{package}.{msg_type}"
                print(f"  Importing LCM module {module_name}...")
                module = importlib.import_module(module_name)
                lcm_class = getattr(module, msg_type)
            except Exception as e:
                print(f"  Error importing LCM class for {schema_type}: {e}")
                print(f"  Will try to continue without decoding...")
                lcm_class = None
            
            # Create topic info
            topic_info = TopicInfo(
                name=base_topic,
                schema_type=schema_type,
                schema=schema,
                lcm_class=lcm_class,
                package=package,
                msg_type=msg_type
            )
            
            # Add topic to our map
            self.topics[topic_name] = topic_info
            
            # Register the topic with Foxglove
            asyncio.run_coroutine_threadsafe(
                self._register_foxglove_channel(topic_info),
                asyncio.get_event_loop()
            )
            
            # Subscribe to the LCM topic
            subscription = self.lc.subscribe(topic_name, self._on_lcm_message)
            self.message_handlers[topic_name] = subscription
            
            print(f"  Subscribed to LCM topic: {topic_name}")
            
        except Exception as e:
            print(f"Error processing topic {topic_name}: {e}")
    
    async def _register_foxglove_channel(self, topic_info):
        """Register a topic with the Foxglove server"""
        try:
            # Format the schema for Foxglove
            channel_info = {
                "topic": topic_info.name,
                "encoding": "json",
                "schemaName": topic_info.schema_type,
                "schemaEncoding": "jsonschema",
                "schema": json.dumps(topic_info.schema),
            }
            
            # Add channel to Foxglove server
            channel_id = await self.server.add_channel(channel_info)
            topic_info.channel_id = channel_id
            
            print(f"  Registered Foxglove channel: {topic_info.name}")
            return channel_id
        except Exception as e:
            print(f"Error registering Foxglove channel for {topic_info.name}: {e}")
    
    def _on_lcm_message(self, channel, data):
        """Called when an LCM message is received"""
        # Find the topic info for this channel
        topic_info = None
        for info in self.topics.values():
            if info.name == channel:
                topic_info = info
                break
                
        if not topic_info or not topic_info.channel_id:
            return
            
        try:
            # Decode the LCM message if we have a class for it
            if topic_info.lcm_class:
                msg = topic_info.lcm_class.decode(data)
                msg_dict = self._lcm_to_dict(msg)
            else:
                # If we can't decode, we'll send raw binary data
                # This won't be very useful to Foxglove but at least we'll see the topic
                msg_dict = {
                    "_raw_data": base64.b64encode(data).decode("ascii")
                }
            
            # Convert the message to JSON and send it
            json_data = json.dumps(msg_dict).encode("utf-8")
            
            # Send via the Foxglove server
            timestamp_ns = int(time.time() * 1e9)
            asyncio.run_coroutine_threadsafe(
                self.server.send_message(topic_info.channel_id, timestamp_ns, json_data),
                asyncio.get_event_loop()
            )
        except Exception as e:
            print(f"Error handling message on {channel}: {e}")
    
    def _lcm_to_dict(self, msg):
        """Convert an LCM message to a dictionary"""
        if isinstance(msg, (int, float, bool, str, type(None))):
            return msg
        elif isinstance(msg, bytes):
            # Convert bytes to base64
            return base64.b64encode(msg).decode("ascii")
        elif isinstance(msg, list):
            return [self._lcm_to_dict(item) for item in msg]
        elif isinstance(msg, dict):
            return {k: self._lcm_to_dict(v) for k, v in msg.items()}
        else:
            # Try to convert a custom LCM message object
            result = {}
            for attr in dir(msg):
                # Skip private attributes and methods
                if attr.startswith('_') or callable(getattr(msg, attr)):
                    continue
                value = getattr(msg, attr)
                result[attr] = self._lcm_to_dict(value)
            return result

class LcmFoxgloveBridgeRunner:
    """Runner class to manage the bridge and server lifecycle"""
    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        self.discoverer = None
        self.lcm_thread = None
        self.running = True
        self.lc = lcm.LCM()
        self.topics = {}
        self.schema_generator = SchemaGenerator()
        self.message_handlers = {}
        
        # For cross-thread communication
        self.topic_queue = asyncio.Queue()
        self.message_queue = asyncio.Queue()
        self.loop = None
        self.server = None
        
    async def run(self):
        """Run the bridge with proper context management for FoxgloveServer"""
        print(f"Starting LCM-Foxglove bridge on {self.host}:{self.port}")
        
        # Store reference to the event loop that will be used for all async operations
        self.loop = asyncio.get_running_loop()
        
        # Start topic discovery
        self.discoverer = LcmTopicDiscoverer(self._on_topic_discovered)
        self.discoverer.start()
        
        # Start LCM handling thread
        self.lcm_thread = threading.Thread(target=self._lcm_thread_func)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
        
        # Create and start Foxglove WebSocket server as a context manager
        async with FoxgloveServer(
            host=self.host,
            port=self.port,
            name="LCM-Foxglove Bridge",
            capabilities=["clientPublish"],
            supported_encodings=["json"],
        ) as server:
            self.server = server
            print(f"WebSocket server started on {self.host}:{self.port}")
            print("Waiting for LCM topics...")
            
            # Start task to process new topics
            topic_processor_task = asyncio.create_task(self._process_topic_queue())
            
            # Start task to process messages
            message_processor_task = asyncio.create_task(self._process_message_queue())
            
            try:
                # Keep running until interrupted
                while self.running:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                print("\nTask cancelled")
            finally:
                # Cancel the processor tasks
                topic_processor_task.cancel()
                message_processor_task.cancel()
                try:
                    await topic_processor_task
                    await message_processor_task
                except asyncio.CancelledError:
                    pass
                self.stop()
    
    async def _process_topic_queue(self):
        """Process new topics from the queue"""
        while True:
            try:
                # Get next topic from the queue
                topic_info = await self.topic_queue.get()
                
                # Only register the topic if the server is available
                if self.server:
                    try:
                        # Format the schema for Foxglove
                        channel_info = {
                            "topic": topic_info.name,
                            "encoding": "json",
                            "schemaName": topic_info.schema_type,
                            "schemaEncoding": "jsonschema",
                            "schema": json.dumps(topic_info.schema),
                        }
                        
                        # Add channel to Foxglove server
                        channel_id = await self.server.add_channel(channel_info)
                        topic_info.channel_id = channel_id
                        
                        print(f"  Registered Foxglove channel: {topic_info.name}")
                    except Exception as e:
                        print(f"  Error registering Foxglove channel for {topic_info.name}: {e}")
                
                # Mark task as done
                self.topic_queue.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error in topic processor: {e}")
    
    async def _process_message_queue(self):
        """Process messages from the queue"""
        while True:
            try:
                # Get next message from the queue
                topic_info, msg_dict, timestamp_ns = await self.message_queue.get()
                
                # Send to Foxglove if server and channel ID are available
                if self.server and topic_info and topic_info.channel_id:
                    try:
                        # Convert the message to JSON and send it
                        json_data = json.dumps(msg_dict).encode("utf-8")
                        await self.server.send_message(topic_info.channel_id, timestamp_ns, json_data)
                    except Exception as e:
                        print(f"  Error sending message for {topic_info.name}: {e}")
                
                # Mark task as done
                self.message_queue.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error in message processor: {e}")
    
    def _lcm_thread_func(self):
        """Thread for handling LCM messages"""
        while self.running:
            try:
                self.lc.handle()
            except Exception as e:
                print(f"Error handling LCM message: {e}")
    
    def _on_topic_discovered(self, topic_name):
        """Called when a new topic is discovered"""
        # Skip if we've already processed this topic
        if topic_name in self.topics:
            return
            
        try:
            print(f"Discovered topic: {topic_name}")
            
            # Extract base topic name and schema type
            base_topic, schema_type = topic_name.split("#", 1)
            package, msg_type = schema_type.split(".", 1)
            
            # Generate schema from ROS message definition
            print(f"  Generating schema for {schema_type}...")
            schema = self.schema_generator.generate_schema(schema_type)
            
            # Try to import the LCM message class
            try:
                module_name = f"lcm_msgs.{package}.{msg_type}"
                print(f"  Importing LCM module {module_name}...")
                module = importlib.import_module(module_name)
                lcm_class = getattr(module, msg_type)
            except Exception as e:
                print(f"  Error importing LCM class for {schema_type}: {e}")
                print(f"  Will try to continue without decoding...")
                lcm_class = None
            
            # Create topic info
            topic_info = TopicInfo(
                name=base_topic,
                schema_type=schema_type,
                schema=schema,
                lcm_class=lcm_class,
                package=package,
                msg_type=msg_type
            )
            
            # Add topic to our map
            self.topics[topic_name] = topic_info
            
            # Queue the topic for registration with Foxglove
            if self.loop:
                self.loop.call_soon_threadsafe(self.topic_queue.put_nowait, topic_info)
            
            # Subscribe to the LCM topic
            subscription = self.lc.subscribe(base_topic, self._on_lcm_message)
            self.message_handlers[base_topic] = subscription
            
            print(f"  Subscribed to LCM topic: {base_topic}")
            
        except Exception as e:
            print(f"Error processing topic {topic_name}: {e}")
    
    def _on_lcm_message(self, channel, data):
        """Called when an LCM message is received"""
        # Find the topic info for this channel
        topic_info = None
        for info in self.topics.values():
            if info.name == channel:
                topic_info = info
                break
                
        if not topic_info or not self.loop:
            return
            
        try:
            # Decode the LCM message if we have a class for it
            if topic_info.lcm_class:
                try:
                    msg = topic_info.lcm_class.decode(data)
                    msg_dict = self._lcm_to_dict(msg)
                    print(f"Received message on {channel} of type {topic_info.schema_type}")
                    # Print message structure for debugging
                    debug_sample = json.dumps(msg_dict)[:200] + "..." if len(json.dumps(msg_dict)) > 200 else json.dumps(msg_dict)
                    print(f"Message preview: {debug_sample}")
                except Exception as e:
                    print(f"Error decoding message on {channel}: {e}")
                    # If decoding fails, use raw data
                    msg_dict = {
                        "_raw_data": base64.b64encode(data).decode("ascii")
                    }
            else:
                # If we can't decode, we'll send raw binary data
                # This won't be very useful to Foxglove but at least we'll see the topic
                msg_dict = {
                    "_raw_data": base64.b64encode(data).decode("ascii")
                }
            
            # Convert the message to JSON and send it
            try:
                json_data = json.dumps(msg_dict).encode("utf-8")
                
                # Send via the message queue
                timestamp_ns = int(time.time() * 1e9)
                self.loop.call_soon_threadsafe(
                    self.message_queue.put_nowait, 
                    (topic_info, msg_dict, timestamp_ns)
                )
            except Exception as e:
                print(f"Error preparing message for {channel}: {e}")
        except Exception as e:
            print(f"Error handling message on {channel}: {e}")
    
    def _lcm_to_dict(self, msg):
        """Convert an LCM message to a dictionary"""
        if isinstance(msg, (int, float, bool, str, type(None))):
            return msg
        elif isinstance(msg, bytes):
            # Convert bytes to base64
            return base64.b64encode(msg).decode("ascii")
        elif isinstance(msg, list):
            return [self._lcm_to_dict(item) for item in msg]
        elif isinstance(msg, dict):
            return {k: self._lcm_to_dict(v) for k, v in msg.items()}
        else:
            # Try to convert a custom LCM message object
            result = {}
            for attr in dir(msg):
                # Skip private attributes and methods
                if attr.startswith('_') or callable(getattr(msg, attr)):
                    continue
                value = getattr(msg, attr)
                result[attr] = self._lcm_to_dict(value)
            return result
            
    def stop(self):
        """Stop the bridge cleanly"""
        print("Stopping LCM-Foxglove bridge")
        self.running = False
        if self.discoverer:
            self.discoverer.stop()

async def main():
    """Main entry point"""
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='LCM to Foxglove WebSocket Bridge')
    parser.add_argument('--host', default='0.0.0.0', help='WebSocket server host')
    parser.add_argument('--port', type=int, default=8765, help='WebSocket server port')
    args = parser.parse_args()
    
    # Create and run the bridge
    bridge = LcmFoxgloveBridgeRunner(host=args.host, port=args.port)
    await bridge.run()

if __name__ == "__main__":
    # Add python_lcm_msgs to path so we can import LCM message modules
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lcm_module_dir = os.path.join(current_dir, "python_lcm_msgs")
    sys.path.append(lcm_module_dir)
    
    # Run the main function
    run_cancellable(main())
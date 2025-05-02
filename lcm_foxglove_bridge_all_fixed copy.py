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
import logging
from typing import Dict, List, Any, Set, Optional
from dataclasses import dataclass

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("lcm_foxglove_bridge")

# Import foxglove-websocket
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

# Constants
ROS_MSGS_DIR = "ros_msgs"
LCM_PYTHON_MODULES_PATH = "python_lcm_msgs/lcm_msgs"

# Hardcoded schemas for Foxglove compatibility
# These are specially formatted to match exactly what Foxglove expects
HARDCODED_SCHEMAS = {
    "sensor_msgs.Image": {
        "foxglove_name": "sensor_msgs/msg/Image",
        "schema": {
            "type": "object",
            "properties": {
                "header": {
                    "type": "object",
                    "properties": {
                        "stamp": {
                            "type": "object",
                            "properties": {
                                "sec": {"type": "integer"},
                                "nanosec": {"type": "integer"}
                            }
                        },
                        "frame_id": {"type": "string"}
                    }
                },
                "height": {"type": "integer"},
                "width": {"type": "integer"},
                "encoding": {"type": "string"},
                "is_bigendian": {"type": "boolean"},
                "step": {"type": "integer"},
                "data": {
                    "type": "string",
                    "contentEncoding": "base64"
                }
            }
        }
    },
    "sensor_msgs.CompressedImage": {
        "foxglove_name": "sensor_msgs/msg/CompressedImage",
        "schema": {
            "type": "object",
            "properties": {
                "header": {
                    "type": "object",
                    "properties": {
                        "stamp": {
                            "type": "object",
                            "properties": {
                                "sec": {"type": "integer"},
                                "nanosec": {"type": "integer"}
                            }
                        },
                        "frame_id": {"type": "string"}
                    }
                },
                "format": {"type": "string"},
                "data": {
                    "type": "string",
                    "contentEncoding": "base64"
                }
            }
        }
    },
    "tf2_msgs.TFMessage": {
        "foxglove_name": "tf2_msgs/msg/TFMessage",
        "schema": {
            "type": "object",
            "properties": {
                "transforms": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "header": {
                                "type": "object",
                                "properties": {
                                    "stamp": {
                                        "type": "object",
                                        "properties": {
                                            "sec": {"type": "integer"},
                                            "nanosec": {"type": "integer"}
                                        }
                                    },
                                    "frame_id": {"type": "string"}
                                }
                            },
                            "child_frame_id": {"type": "string"},
                            "transform": {
                                "type": "object",
                                "properties": {
                                    "translation": {
                                        "type": "object",
                                        "properties": {
                                            "x": {"type": "number"},
                                            "y": {"type": "number"},
                                            "z": {"type": "number"}
                                        }
                                    },
                                    "rotation": {
                                        "type": "object",
                                        "properties": {
                                            "x": {"type": "number"},
                                            "y": {"type": "number"},
                                            "z": {"type": "number"},
                                            "w": {"type": "number"}
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    },
    "sensor_msgs.PointCloud2": {
        "foxglove_name": "sensor_msgs/msg/PointCloud2",
        "schema": {
            "type": "object",
            "properties": {
                "header": {
                    "type": "object",
                    "properties": {
                        "stamp": {
                            "type": "object",
                            "properties": {
                                "sec": {"type": "integer"},
                                "nanosec": {"type": "integer"}
                            },
                            "required": ["sec", "nanosec"]
                        },
                        "frame_id": {"type": "string"}
                    },
                    "required": ["stamp", "frame_id"]
                },
                "height": {"type": "integer"},
                "width": {"type": "integer"},
                "fields": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "name": {"type": "string"},
                            "offset": {"type": "integer"},
                            "datatype": {"type": "integer"},
                            "count": {"type": "integer"},
                        },
                        "required": ["name","offset","datatype","count"]
                    }
                },
                "is_bigendian": {"type": "boolean"},
                "point_step": {"type": "integer"},
                "row_step": {"type": "integer"},
                "data": {
                    "type": "string",
                    "contentEncoding": "base64"
                },
                "is_dense": {"type": "boolean"}
            },
            "required": [
                "header","height","width","fields",
                "is_bigendian","point_step","row_step",
                "data","is_dense"
            ]
        }
    }
}

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
            "nanosec": {"type": "integer"}
        },
        "required": ["sec", "nanosec"]
    },
    "duration": {
        "type": "object",
        "properties": {
            "sec": {"type": "integer"},
            "nanosec": {"type": "integer"}
        },
        "required": ["sec", "nanosec"]
    },
}

@dataclass
class TopicInfo:
    """Information about an LCM topic with schema"""
    name: str  # Base topic name (without schema) for Foxglove
    full_topic_name: str  # Full LCM topic name including schema annotation
    schema_type: str  # Schema type (e.g., "sensor_msgs.Image")
    schema: dict  # JSON schema
    channel_id: Optional[ChannelId] = None  # Foxglove channel ID
    lcm_class: Any = None  # LCM message class
    package: str = ""  # ROS package name
    msg_type: str = ""  # ROS message type
    foxglove_schema_name: str = ""  # Schema name in Foxglove format

class SchemaGenerator:
    """Generates JSON schemas from ROS message definitions"""
    def __init__(self):
        self.schema_cache = {}
        
    def generate_schema(self, schema_type):
        """Generate a JSON schema for the given schema type (e.g., 'sensor_msgs.Image')"""
        # Check if we have a hardcoded schema for this type
        if schema_type in HARDCODED_SCHEMAS:
            logger.info(f"Using hardcoded schema for {schema_type}")
            return HARDCODED_SCHEMAS[schema_type]["schema"]
            
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
                            "nanosec": {"type": "integer"}  # Use nanosec instead of nsec for Foxglove compatibility
                        },
                        "required": ["sec", "nanosec"]
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
                logger.error(f"Error processing complex type {field_type}: {e}")
                # Return a placeholder schema
                return {"type": "object", "description": f"Error: could not process type {field_type}"}

class LcmTopicDiscoverer:
    """Discovers LCM topics and their schemas"""
    def __init__(self, callback, schema_map=None):
        """
        Initialize the topic discoverer
        
        Args:
            callback: Function to call when a new topic is discovered
            schema_map: Optional dict mapping bare topic names to schema types
        """
        self.lc = lcm.LCM()
        self.callback = callback
        self.topics = set()
        self.running = True
        self.thread = threading.Thread(target=self._discovery_thread)
        self.mutex = threading.Lock()
        self.schema_map = schema_map or {}
        
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
                logger.error(f"Error in LCM discovery: {e}")
    
    def _on_any_message(self, channel, data):
        """Callback for any LCM message during discovery"""
        with self.mutex:
            if channel not in self.topics:
                # New topic found
                self.topics.add(channel)
                
                # Check if the topic has schema information
                if "#" in channel:
                    # Topic already has schema info in the name
                    try:
                        self.callback(channel)
                    except Exception as e:
                        logger.error(f"Error processing discovered topic {channel}: {e}")
                elif channel in self.schema_map:
                    # We have schema info in our map
                    schema_type = self.schema_map[channel]
                    annotated_channel = f"{channel}#{schema_type}"
                    try:
                        self.callback(annotated_channel)
                    except Exception as e:
                        logger.error(f"Error processing mapped topic {channel} with schema {schema_type}: {e}")

class LcmFoxgloveBridgeRunner:
    """Runner class to manage the bridge and server lifecycle"""
    def __init__(self, host="0.0.0.0", port=8765, schema_map=None, debug=False):
        self.host = host
        self.port = port
        self.discoverer = None
        self.lcm_thread = None
        self.running = True
        self.lc = lcm.LCM()
        self.topics = {}
        self.schema_generator = SchemaGenerator()
        self.message_handlers = {}
        self.schema_map = schema_map or {}
        self.debug = debug
        
        # For cross-thread communication
        self.topic_queue = asyncio.Queue()
        self.message_queue = asyncio.Queue()
        self.loop = None
        self.server = None
        
        # Configure verbose debugging
        if debug:
            logger.setLevel(logging.DEBUG)
        
    async def run(self):
        """Run the bridge with proper context management for FoxgloveServer"""
        logger.info(f"Starting LCM-Foxglove bridge on {self.host}:{self.port}")
        
        # Store reference to the event loop that will be used for all async operations
        self.loop = asyncio.get_running_loop()
        
        # Start topic discovery
        self.discoverer = LcmTopicDiscoverer(self._on_topic_discovered, self.schema_map)
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
            logger.info(f"WebSocket server started on {self.host}:{self.port}")
            logger.info("Waiting for LCM topics...")
            
            # Start task to process new topics
            topic_processor_task = asyncio.create_task(self._process_topic_queue())
            
            # Start task to process messages
            message_processor_task = asyncio.create_task(self._process_message_queue())
            
            try:
                # Keep running until interrupted
                while self.running:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                logger.info("\nTask cancelled")
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
                        # Get the foxglove schema name (either from hardcoded schemas or standard conversion)
                        if topic_info.schema_type in HARDCODED_SCHEMAS:
                            foxglove_schema_name = HARDCODED_SCHEMAS[topic_info.schema_type]["foxglove_name"]
                        else:
                            # Convert from package.MsgType to package/msg/MsgType format for Foxglove
                            foxglove_schema_name = topic_info.schema_type.replace(".", "/msg/")
                        
                        # Store for reference
                        topic_info.foxglove_schema_name = foxglove_schema_name
                        
                        # Format the schema for Foxglove
                        channel_info = {
                            "topic": topic_info.name,
                            "encoding": "json",
                            "schemaName": foxglove_schema_name,
                            "schemaEncoding": "jsonschema",
                            "schema": json.dumps(topic_info.schema),
                        }
                        
                        # Add channel to Foxglove server
                        channel_id = await self.server.add_channel(channel_info)
                        topic_info.channel_id = channel_id
                        
                        logger.info(f"Registered Foxglove channel: {topic_info.name} with schema: {foxglove_schema_name}")
                        
                        # Special handling for TF topics - make sure they're properly initialized
                        if topic_info.schema_type.lower() == "tf2_msgs.tfmessage":
                            logger.info(f"TF topic registered with channel ID: {channel_id}")
                        # Special handling for PointCloud2 topics
                        elif topic_info.schema_type.lower() == "sensor_msgs.pointcloud2":
                            logger.info(f"PointCloud2 topic registered with channel ID: {channel_id}")
                    except Exception as e:
                        logger.error(f"Error registering Foxglove channel for {topic_info.name}: {e}")
                        import traceback
                        traceback.print_exc()
                
                # Mark task as done
                self.topic_queue.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in topic processor: {e}")
                import traceback
                traceback.print_exc()
    
    async def _process_message_queue(self):
        """Process messages from the queue"""
        while True:
            try:
                # Get next message from the queue
                topic_info, msg_dict, timestamp_ns = await self.message_queue.get()
                
                # Send to Foxglove if server and channel ID are available
                if self.server and topic_info and topic_info.channel_id is not None:
                    try:
                        # Extra debug for TF messages
                        if topic_info.schema_type.lower() == "tf2_msgs.tfmessage":
                            logger.info(f"Sending TF message with {len(msg_dict.get('transforms', []))} transforms to channel ID: {topic_info.channel_id}")
                        # Extra debug for PointCloud2 messages
                        elif topic_info.schema_type.lower() == "sensor_msgs.pointcloud2":
                            logger.info(f"Sending PointCloud2 message with {len(msg_dict.get('data', ''))} bytes of data")
                        
                        # Convert the message to JSON and send it
                        json_data = json.dumps(msg_dict).encode("utf-8")
                        
                        # Always log important messages being sent, even in non-debug mode
                        if topic_info.schema_type.lower() in ["tf2_msgs.tfmessage", "sensor_msgs.pointcloud2"]:
                            logger.info(f"Sending message on channel {topic_info.name} (ID: {topic_info.channel_id}) with timestamp {timestamp_ns}")
                        elif self.debug:
                            logger.debug(f"Sending message on channel {topic_info.name} with timestamp {timestamp_ns}: {json_data[:100]}...")
                        
                        await self.server.send_message(topic_info.channel_id, timestamp_ns, json_data)
                    except Exception as e:
                        logger.error(f"Error sending message for {topic_info.name}: {e}")
                        import traceback
                        traceback.print_exc()
                else:
                    if topic_info and topic_info.channel_id is None:
                        logger.warning(f"Topic {topic_info.name} has no channel ID, can't send message")
                
                # Mark task as done
                self.message_queue.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in message processor: {e}")
                import traceback
                traceback.print_exc()
    
    def _lcm_thread_func(self):
        """Thread for handling LCM messages"""
        while self.running:
            try:
                self.lc.handle()
            except Exception as e:
                logger.error(f"Error handling LCM message: {e}")
    
    def _on_topic_discovered(self, topic_name):
        """Called when a new topic is discovered"""
        # Skip if we've already processed this topic
        if topic_name in self.topics:
            return
            
        try:
            logger.info(f"Discovered topic: {topic_name}")
            
            # Extract base topic name and schema type
            base_topic, schema_type = topic_name.split("#", 1)
            package, msg_type = schema_type.split(".", 1)
            
            # Generate schema from ROS message definition
            logger.info(f"Generating schema for {schema_type}...")
            schema = self.schema_generator.generate_schema(schema_type)
            
            # Try to import the LCM message class
            try:
                module_name = f"lcm_msgs.{package}.{msg_type}"
                logger.info(f"Importing LCM module {module_name}...")
                module = importlib.import_module(module_name)
                lcm_class = getattr(module, msg_type)
            except Exception as e:
                logger.warning(f"Error importing LCM class for {schema_type}: {e}")
                logger.warning(f"Will try to continue without decoding...")
                lcm_class = None
            
            # Create topic info
            topic_info = TopicInfo(
                name=base_topic,
                full_topic_name=topic_name,
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
            
            # Subscribe to the FULL LCM topic name (including schema annotation)
            # This is critical because in LCM the full string is the actual topic name
            subscription = self.lc.subscribe(topic_name, self._on_lcm_message)
            self.message_handlers[topic_name] = subscription
            
            logger.info(f"Subscribed to LCM topic: {topic_name}")
            
        except Exception as e:
            logger.error(f"Error processing topic {topic_name}: {e}")
            import traceback
            traceback.print_exc()
    
    def _on_lcm_message(self, channel, data):
        """Called when an LCM message is received"""
        # The channel parameter is the full topic name (with schema annotation)
        # We need to look up the topic info directly by this key
        topic_info = self.topics.get(channel)
        
        if not topic_info or not self.loop:
            if self.debug:
                logger.warning(f"Received message for unknown channel: {channel}")
            return
            
        try:
            # Decode the LCM message if we have a class for it
            if topic_info.lcm_class:
                try:
                    msg = topic_info.lcm_class.decode(data)
                    
                    # Apply special formatting based on message type (case-insensitive)
                    schema_type_lower = topic_info.schema_type.lower()
                    if schema_type_lower == "sensor_msgs.image":
                        msg_dict = self._format_image_msg(msg, topic_info)
                    elif schema_type_lower == "sensor_msgs.compressedimage":
                        msg_dict = self._format_compressed_image_msg(msg, topic_info)
                    elif schema_type_lower == "tf2_msgs.tfmessage":
                        msg_dict = self._format_tf_msg(msg, topic_info)
                        # Special handling for TF messages - make sure they have transforms
                        if not msg_dict.get("transforms"):
                            logger.warning("TF message has no transforms, skipping")
                            return
                    elif schema_type_lower == "sensor_msgs.jointstate":
                        msg_dict = self._format_joint_state_msg(msg, topic_info)
                    elif schema_type_lower == "sensor_msgs.pointcloud2":
                        msg_dict = self._format_pointcloud2_msg(msg, topic_info)
                        # Skip empty point clouds
                        if not msg_dict.get("data"):
                            logger.warning(f"PointCloud2 message has no data, adding default field definitions and empty data")
                            # Add default field definitions
                            msg_dict["fields"] = [
                                {"name": "x", "offset": 0, "datatype": 7, "count": 1},
                                {"name": "y", "offset": 4, "datatype": 7, "count": 1},
                                {"name": "z", "offset": 8, "datatype": 7, "count": 1}
                            ]
                            # Add empty data array
                            msg_dict["data"] = ""
                    else:
                        # Generic conversion for other message types
                        msg_dict = self._lcm_to_dict(msg)
                    
                    if self.debug:
                        logger.debug(f"Received message on {channel} of type {topic_info.schema_type}")
                        # Print message preview for debugging
                        debug_sample = json.dumps(msg_dict)[:100] + "..." if len(json.dumps(msg_dict)) > 100 else json.dumps(msg_dict)
                        logger.debug(f"Message preview: {debug_sample}")
                    
                    # Special handling for TF messages
                    if schema_type_lower == "tf2_msgs.tfmessage":
                        # Log TF transforms for debugging
                        if self.debug:
                            logger.debug(f"TF message parsed with {len(msg_dict.get('transforms', []))} transforms")
                    
                    # Special handling for PointCloud2 messages
                    elif schema_type_lower == "sensor_msgs.pointcloud2":
                        if self.debug:
                            logger.debug(f"PointCloud2 message with fields: {msg_dict.get('fields', [])}")
                            logger.debug(f"PointCloud2 dimensions: {msg_dict.get('width', 0)} x {msg_dict.get('height', 0)}")
                except Exception as e:
                    logger.error(f"Error decoding message on {channel}: {e}")
                    import traceback
                    traceback.print_exc()
                    # If decoding fails, use raw data
                    msg_dict = {
                        "_raw_data": base64.b64encode(data).decode("ascii")
                    }
            else:
                # If we can't decode, we'll send raw binary data
                if self.debug:
                    logger.debug(f"Received message on {channel} but no decoder available, using raw data")
                msg_dict = {
                    "_raw_data": base64.b64encode(data).decode("ascii")
                }
            
            # Send via the message queue
            timestamp_ns = int(time.time() * 1e9)
            self.loop.call_soon_threadsafe(
                self.message_queue.put_nowait, 
                (topic_info, msg_dict, timestamp_ns)
            )
        except Exception as e:
            logger.error(f"Error handling message on {channel}: {e}")
            import traceback
            traceback.print_exc()
    
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
            
            # First gather all attributes and their types
            attribute_info = {}
            for attr in dir(msg):
                if attr.startswith('_') or callable(getattr(msg, attr)):
                    continue
                    
                # Store attribute and check if it's a length field
                if attr.endswith('_length'):
                    base_attr = attr[:-7]  # Remove '_length' suffix
                    attribute_info[base_attr] = attribute_info.get(base_attr, {})
                    attribute_info[base_attr]['has_length'] = True
                    attribute_info[base_attr]['length_value'] = getattr(msg, attr)
                else:
                    attribute_info[attr] = attribute_info.get(attr, {})
                    attribute_info[attr]['value'] = getattr(msg, attr)
            
            # Process the gathered attributes
            for attr, info in attribute_info.items():
                if 'value' not in info:
                    continue  # Skip length-only entries
                    
                try:
                    value = info['value']
                    
                    # Handle arrays with corresponding length fields
                    if info.get('has_length', False) and isinstance(value, list):
                        length = info['length_value']
                        if isinstance(length, int) and length >= 0 and length <= len(value):
                            value = value[:length]
                    
                    # Recursively convert the value
                    result[attr] = self._lcm_to_dict(value)
                except Exception as e:
                    logger.error(f"Error converting attribute {attr}: {e}")
                    result[attr] = None
                    
            return result
    
    def _format_image_msg(self, msg, topic_info=None):
        """Format a sensor_msgs/Image message for Foxglove"""
        try:
            # Get header
            header = self._get_header_dict(msg.header)
            
            # For Image messages, we need to encode the data as base64
            data_length = getattr(msg, "data_length", len(msg.data) if hasattr(msg, "data") else 0)
            
            # Convert potentially incompatible encoding
            encoding = msg.encoding if hasattr(msg, "encoding") and msg.encoding else "rgb8"
            # Foxglove might need rgb8 instead of bgr8
            if encoding.lower() == "bgr8":
                # Here's the critical fix for black images - we need to convert BGR to RGB
                # In this particular case, we're just changing the encoding string
                encoding = "rgb8"  # Tell Foxglove this is rgb8 format
                if self.debug:
                    logger.debug(f"Converting BGR8 to RGB8 image encoding")
            
            if self.debug:
                logger.debug(f"Image data length: {data_length}, height: {msg.height}, width: {msg.width}, encoding: {encoding}")
            
            if hasattr(msg, "data") and data_length > 0:
                # Get image data as bytes
                image_data_bytes = msg.data[:data_length]
                
                # For debugging - check if the image data is valid
                if self.debug:
                    logger.debug(f"Image raw data length: {len(image_data_bytes)} bytes")
                    logger.debug(f"First 20 bytes: {[b for b in image_data_bytes[:20]]}")
                
                # Convert to base64
                image_data = base64.b64encode(image_data_bytes).decode("ascii")
            else:
                # If no data, return empty string
                image_data = ""
            
            # Return properly formatted message dict for Foxglove
            return {
                "header": header,
                "height": int(msg.height),
                "width": int(msg.width),
                "encoding": encoding,
                "is_bigendian": bool(msg.is_bigendian),
                "step": int(msg.step),
                "data": image_data
            }
        except Exception as e:
            logger.error(f"Error formatting Image message: {e}")
            import traceback
            traceback.print_exc()
            return self._lcm_to_dict(msg)  # Fallback to generic conversion
    
    def _format_compressed_image_msg(self, msg, topic_info=None):
        """Format a sensor_msgs/CompressedImage message for Foxglove"""
        try:
            # Get header
            header = self._get_header_dict(msg.header)
            
            # For CompressedImage messages, format must be jpg or png
            image_format = msg.format.lower() if hasattr(msg, "format") and msg.format else "jpeg"
            
            # Convert data to base64
            data_length = getattr(msg, "data_length", len(msg.data) if hasattr(msg, "data") else 0)
            
            if self.debug:
                logger.debug(f"CompressedImage format: {image_format}, data length: {data_length}")
            
            if hasattr(msg, "data") and data_length > 0:
                # Get image data as bytes
                image_data_bytes = msg.data[:data_length]
                
                # For debugging
                if self.debug:
                    logger.debug(f"CompressedImage raw data length: {len(image_data_bytes)} bytes")
                    if len(image_data_bytes) > 20:
                        logger.debug(f"First 20 bytes: {[b for b in image_data_bytes[:20]]}")
                
                # Convert to base64
                image_data = base64.b64encode(image_data_bytes).decode("ascii")
            else:
                # If no data, return empty string
                image_data = ""
            
            # Return properly formatted message for Foxglove
            return {
                "header": header, 
                "format": image_format,
                "data": image_data
            }
        except Exception as e:
            logger.error(f"Error formatting CompressedImage message: {e}")
            import traceback
            traceback.print_exc()
            return self._lcm_to_dict(msg)  # Fallback to generic conversion
    
    def _format_tf_msg(self, msg, topic_info=None):
        """Format a tf2_msgs/TFMessage for Foxglove"""
        try:
            # Get the transforms array with correct length
            transforms_length = getattr(msg, "transforms_length", 0)
            transforms = []
            
            if self.debug:
                logger.debug(f"Processing TF message with {transforms_length} transforms")
                
                # Print some detailed info about the TF message
                logger.debug(f"TF message attributes: {[attr for attr in dir(msg) if not attr.startswith('_') and not callable(getattr(msg, attr))]}")
                if hasattr(msg, "transforms") and transforms_length > 0 and len(msg.transforms) > 0:
                    logger.debug(f"TF message has {len(msg.transforms)} transforms in the list")
                    tf_sample = msg.transforms[0]
                    logger.debug(f"First transform child frame: {tf_sample.child_frame_id if hasattr(tf_sample, 'child_frame_id') else 'unknown'}")
            
            # Process each transform in the array
            if hasattr(msg, "transforms") and transforms_length > 0:
                for i in range(min(transforms_length, len(msg.transforms))):
                    transform = msg.transforms[i]
                    transform_dict = self._format_transform_stamped(transform)
                    if transform_dict:
                        transforms.append(transform_dict)
                        
                        if self.debug and i == 0:
                            # Just print the first one to avoid log spam
                            logger.debug(f"First transform: {json.dumps(transform_dict)[:200]}...")
            
            # Return properly formatted message
            result = {"transforms": transforms}
            
            # Log the result if we're seeing TF issues
            if not transforms:
                logger.warning("Warning: No transforms found in TF message")
            elif self.debug:
                logger.debug(f"Formatted TF message with {len(transforms)} transforms")
                
            return result
            
        except Exception as e:
            logger.error(f"Error formatting TFMessage: {e}")
            import traceback
            traceback.print_exc()
            return {"transforms": []}  # Return empty transforms array on error
    
    def _format_transform_stamped(self, transform):
        """Format a geometry_msgs/TransformStamped message for Foxglove"""
        try:
            # Format header
            header = self._get_header_dict(transform.header)
            
            # The format must exactly match TransformStamped message expected by Foxglove:
            # https://foxglove.dev/docs/studio/visualization/3d-panel
            
            if self.debug:
                # Print transform details
                if hasattr(transform, "transform"):
                    t = transform.transform
                    if hasattr(t, "translation"):
                        logger.debug(f"Translation: ({getattr(t.translation, 'x', 0)}, {getattr(t.translation, 'y', 0)}, {getattr(t.translation, 'z', 0)})")
                    if hasattr(t, "rotation"):
                        logger.debug(f"Rotation: ({getattr(t.rotation, 'x', 0)}, {getattr(t.rotation, 'y', 0)}, {getattr(t.rotation, 'z', 0)}, {getattr(t.rotation, 'w', 1)})")
            
            # Check for required attributes
            if not hasattr(transform, "transform"):
                logger.warning("Warning: TransformStamped missing 'transform' attribute")
                return None
                
            if not hasattr(transform.transform, "translation") or not hasattr(transform.transform, "rotation"):
                logger.warning("Warning: Transform missing translation or rotation")
                return None
            
            # Format translation (defaulting to zeros if missing)
            translation = {
                "x": float(getattr(transform.transform.translation, "x", 0.0)),
                "y": float(getattr(transform.transform.translation, "y", 0.0)),
                "z": float(getattr(transform.transform.translation, "z", 0.0))
            }
            
            # Format rotation (defaulting to identity if missing)
            rotation = {
                "x": float(getattr(transform.transform.rotation, "x", 0.0)),
                "y": float(getattr(transform.transform.rotation, "y", 0.0)),
                "z": float(getattr(transform.transform.rotation, "z", 0.0)),
                "w": float(getattr(transform.transform.rotation, "w", 1.0))
            }
            
            # Get child frame id
            child_frame_id = transform.child_frame_id
            if isinstance(child_frame_id, bytes):
                child_frame_id = child_frame_id.decode('utf-8', errors='replace')
            
            # Return formatted transform (exactly as Foxglove TF expects)
            return {
                "header": header,
                "child_frame_id": child_frame_id,
                "transform": {
                    "translation": translation,
                    "rotation": rotation
                }
            }
        except Exception as e:
            logger.error(f"Error formatting TransformStamped: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _format_joint_state_msg(self, msg, topic_info=None):
        """Format a sensor_msgs/JointState message for Foxglove"""
        try:
            # Format the header
            header = self._get_header_dict(msg.header)
            
            # Get array lengths
            name_length = getattr(msg, "name_length", 0)
            position_length = getattr(msg, "position_length", 0)
            velocity_length = getattr(msg, "velocity_length", 0)
            effort_length = getattr(msg, "effort_length", 0)
            
            # Format arrays with correct lengths
            names = msg.name[:name_length] if hasattr(msg, "name") and name_length > 0 else []
            positions = msg.position[:position_length] if hasattr(msg, "position") and position_length > 0 else []
            velocities = msg.velocity[:velocity_length] if hasattr(msg, "velocity") and velocity_length > 0 else []
            efforts = msg.effort[:effort_length] if hasattr(msg, "effort") and effort_length > 0 else []
            
            # Convert name list items from bytes to strings if needed
            names = [name.decode('utf-8', errors='replace') if isinstance(name, bytes) else name for name in names]
            
            # Convert array elements to Python primitives
            positions = [float(p) for p in positions]
            velocities = [float(v) for v in velocities] 
            efforts = [float(e) for e in efforts]
            
            # Return properly formatted message for Foxglove
            return {
                "header": header,
                "name": names,
                "position": positions,
                "velocity": velocities,
                "effort": efforts
            }
        except Exception as e:
            logger.error(f"Error formatting JointState: {e}")
            import traceback
            traceback.print_exc()
            return {"header": self._get_header_dict(msg.header), "name": [], "position": [], "velocity": [], "effort": []}

    def _format_pointcloud2_msg(self, msg, topic_info=None):
        """Format a sensor_msgs/PointCloud2 message for Foxglove"""
        try:
            # Format the header
            header = self._get_header_dict(msg.header)
            
            # Get fields with correct length
            fields_length = getattr(msg, "fields_length", 0)
            fields = []
            
            # Always log information about PointCloud2 for debugging
            logger.info(f"Processing PointCloud2 message with {fields_length} fields, dimensions: {msg.width}x{msg.height}")
            logger.info(f"Point step: {getattr(msg, 'point_step', 0)}, data length: {len(msg.data) if hasattr(msg, 'data') else 0}")
            
            # Basic validation check to avoid the "not a multiple of point_step" error
            data_len = len(msg.data) if hasattr(msg, 'data') else 0
            point_step = int(msg.point_step) if hasattr(msg, 'point_step') else 16
            
            if data_len % point_step != 0:
                logger.warning(f"PointCloud2 data length {data_len} is not a multiple of point_step {point_step}!")
                # Adjust the data to make it a multiple of point_step
                missing_bytes = point_step - (data_len % point_step)
                logger.info(f"Adding {missing_bytes} padding bytes to make data a multiple of point_step")
                padded_data = msg.data + bytes(missing_bytes)
                msg.data = padded_data
                data_len = len(msg.data)
                
                # Also adjust width to match the new number of points
                adjusted_points = data_len // point_step
                logger.info(f"Adjusted point count from {msg.width} to {adjusted_points}")
                msg.width = adjusted_points
            
            # Extract field definitions
            if hasattr(msg, "fields") and fields_length > 0:
                for i in range(min(fields_length, len(msg.fields))):
                    field = msg.fields[i]
                    field_dict = {
                        "name": field.name.decode('utf-8', errors='replace') if isinstance(field.name, bytes) else field.name,
                        "offset": int(field.offset),
                        "datatype": int(field.datatype),
                        "count": int(field.count)
                    }
                    fields.append(field_dict)
                    logger.info(f"Field: {field_dict['name']}, offset: {field_dict['offset']}, type: {field_dict['datatype']}")
            else:
                # If no fields, provide exactly 4 fields: XYZ + intensity
                logger.warning("No fields in PointCloud2 message, adding default XYZI fields")
                fields = [
                    {"name": "x", "offset": 0, "datatype": 7, "count": 1},  # Float32
                    {"name": "y", "offset": 4, "datatype": 7, "count": 1},  # Float32
                    {"name": "z", "offset": 8, "datatype": 7, "count": 1},  # Float32
                    {"name": "intensity", "offset": 12, "datatype": 7, "count": 1}  # Float32
                ]
            
            # Ensure we have exactly 4 fields (x, y, z, intensity/rgb)
            if len(fields) < 4:
                logger.warning(f"PointCloud2 has only {len(fields)} fields, adding intensity field")
                fields.append({"name": "intensity", "offset": 12, "datatype": 7, "count": 1})
            
            # Convert data to base64
            if hasattr(msg, "data") and len(msg.data) > 0:
                # Get point cloud data as bytes - use ALL data available
                cloud_data_bytes = msg.data
                
                # Check if the data size is correct
                expected_size = int(msg.width) * int(msg.height) * point_step
                actual_size = len(cloud_data_bytes)
                
                logger.info(f"PointCloud2 data size: expected {expected_size}, actual {actual_size}")
                
                # Convert to base64
                cloud_data = base64.b64encode(cloud_data_bytes).decode("ascii")
                logger.info(f"PointCloud2 data converted: {len(cloud_data_bytes)} bytes -> {len(cloud_data)} base64 chars")
            else:
                logger.warning("No data in PointCloud2 message")
                # Create a small point cloud with a single point at origin
                import struct
                # Pack a single point (0,0,0,1.0) as four float32 values (xyz + intensity)
                cloud_data_bytes = struct.pack("<ffff", 0, 0, 0, 1.0)
                cloud_data = base64.b64encode(cloud_data_bytes).decode("ascii")
                # Set width to 1 if no data
                msg.width = 1
                msg.height = 1
            
            # Return properly formatted message for Foxglove
            return {
                "header": header,
                "height": int(msg.height) if hasattr(msg, "height") else 1,
                "width": int(msg.width) if hasattr(msg, "width") else 1,
                "fields": fields,
                "is_bigendian": bool(msg.is_bigendian) if hasattr(msg, "is_bigendian") else False,
                "point_step": point_step,
                "row_step": point_step * int(msg.width) if hasattr(msg, "width") else point_step,
                "data": cloud_data,
                "is_dense": bool(msg.is_dense) if hasattr(msg, "is_dense") else True
            }
        except Exception as e:
            logger.error(f"Error formatting PointCloud2: {e}")
            import traceback
            traceback.print_exc()
            return self._lcm_to_dict(msg)  # Fallback to generic conversion
    
    def _get_header_dict(self, header):
        """Extract a properly formatted header dictionary from a ROS Header"""
        try:
            # Standard ROS header has seq, stamp, and frame_id
            stamp = {}
            
            # Handle stamp which might be a struct or separate sec/nsec fields
            if hasattr(header, "stamp") and not isinstance(header.stamp, (int, float)):
                if hasattr(header.stamp, "sec") and hasattr(header.stamp, "nsec"):
                    stamp = {
                        "sec": int(header.stamp.sec) if hasattr(header.stamp, "sec") else 0,
                        "nanosec": int(header.stamp.nsec) if hasattr(header.stamp, "nsec") else 0
                    }
                else:
                    # Handle builtin_interfaces/Time which might use nanosec instead of nsec
                    stamp = {
                        "sec": int(header.stamp.sec) if hasattr(header.stamp, "sec") else 0,
                        "nanosec": int(
                            header.stamp.nanosec if hasattr(header.stamp, "nanosec") 
                            else (header.stamp.nsec if hasattr(header.stamp, "nsec") else 0)
                        )
                    }
            elif hasattr(header, "sec") and hasattr(header, "nsec"):
                # Some messages have sec/nsec directly in the header
                stamp = {
                    "sec": int(header.sec),
                    "nanosec": int(header.nsec)
                }
            else:
                # Default to current time if no valid stamp found
                now = time.time()
                stamp = {
                    "sec": int(now),
                    "nanosec": int((now % 1) * 1e9)
                }
                
            # Ensure frame_id is a string (convert bytes if needed)
            frame_id = header.frame_id if hasattr(header, "frame_id") else ""
            if isinstance(frame_id, bytes):
                frame_id = frame_id.decode('utf-8', errors='replace')
                
            return {
                "seq": int(header.seq) if hasattr(header, "seq") else 0,
                "stamp": stamp,
                "frame_id": frame_id
            }
        except Exception as e:
            logger.error(f"Error formatting header: {e}")
            import traceback
            traceback.print_exc()
            # Return a minimal valid header
            now = time.time()
            return {
                "seq": 0,
                "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                "frame_id": ""
            }
            
    def stop(self):
        """Stop the bridge cleanly"""
        logger.info("Stopping LCM-Foxglove bridge")
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
    parser.add_argument('--debug', action='store_true', help='Enable verbose debug output')
    parser.add_argument('--map-file', type=str, help='JSON file mapping topic names to schema types')
    args = parser.parse_args()
    
    # Configure debug logging
    if args.debug:
        logger.setLevel(logging.DEBUG)
    
    # Load schema map if provided
    schema_map = {}
    if args.map_file:
        try:
            with open(args.map_file, 'r') as f:
                schema_map = json.load(f)
            logger.info(f"Loaded schema map from {args.map_file} with {len(schema_map)} entries")
        except Exception as e:
            logger.error(f"Error loading schema map file: {e}")
    
    # Create and run the bridge
    bridge = LcmFoxgloveBridgeRunner(
        host=args.host, 
        port=args.port, 
        schema_map=schema_map,
        debug=args.debug
    )
    await bridge.run()

if __name__ == "__main__":
    # Add python_lcm_msgs to path so we can import LCM message modules
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lcm_module_dir = os.path.join(current_dir, "python_lcm_msgs")
    sys.path.append(lcm_module_dir)
    
    # Run the main function
    run_cancellable(main())
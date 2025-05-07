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
import queue
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Set, Optional, Tuple, Callable
from dataclasses import dataclass
import numpy as np
from collections import defaultdict

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
DEFAULT_THREAD_POOL_SIZE = 8
MESSAGE_BATCH_SIZE = 10
MAX_QUEUE_SIZE = 100

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
    "sensor_msgs.JointState": {
        "foxglove_name": "sensor_msgs/msg/JointState",
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
                "name": {
                    "type": "array",
                    "items": {"type": "string"}
                },
                "position": {
                    "type": "array",
                    "items": {"type": "number"}
                },
                "velocity": {
                    "type": "array",
                    "items": {"type": "number"}
                },
                "effort": {
                    "type": "array",
                    "items": {"type": "number"}
                }
            },
            "required": ["header", "name", "position", "velocity", "effort"]
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
    last_sent_timestamp: float = 0.0  # Time the last message was sent (for throttling)
    message_count: int = 0  # Number of messages received
    is_high_frequency: bool = False  # Flag for topics that send many messages
    cache_hash: Optional[int] = None  # Hash of the last message (for message deduplication)
    rate_limit_ms: int = 0  # Rate limit in milliseconds (0 = no limit)

@dataclass
class LcmMessage:
    """Container for an LCM message"""
    topic_info: TopicInfo
    data: bytes
    receive_time: float
    priority: int = 0  # Higher priority will be processed first
    
    def __lt__(self, other):
        # Compare based on priority (higher priority comes first)
        if not isinstance(other, LcmMessage):
            return NotImplemented
        return self.priority > other.priority  # Reversed for higher priority first
    
    def __gt__(self, other):
        if not isinstance(other, LcmMessage):
            return NotImplemented
        return self.priority < other.priority  # Reversed for higher priority first
    
    def __eq__(self, other):
        if not isinstance(other, LcmMessage):
            return NotImplemented
        return self.priority == other.priority

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
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop the discovery thread"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)  # Wait up to 2 seconds for clean shutdown
            
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
                logger.info(f"Discovered new LCM topic: {channel}")
                
                # Special debugging for joint states topic
                if "joint_state" in channel.lower():
                    logger.info(f"Found joint states topic: {channel}")
                
                # Check if the topic has schema information
                if "#" in channel:
                    # Topic already has schema info in the name
                    try:
                        logger.info(f"Processing topic with embedded schema: {channel}")
                        self.callback(channel)
                    except Exception as e:
                        logger.error(f"Error processing discovered topic {channel}: {e}")
                elif channel in self.schema_map:
                    # We have schema info in our map
                    schema_type = self.schema_map[channel]
                    annotated_channel = f"{channel}#{schema_type}"
                    try:
                        logger.info(f"Mapping topic {channel} to {annotated_channel}")
                        self.callback(annotated_channel)
                    except Exception as e:
                        logger.error(f"Error processing mapped topic {channel} with schema {schema_type}: {e}")
                else:
                    # No schema information available
                    logger.warning(f"No schema information for topic: {channel}")
                    if "joint_state" in channel.lower():
                        # Auto-map joint states topic for debugging
                        schema_type = "sensor_msgs.JointState"
                        annotated_channel = f"{channel}#{schema_type}"
                        logger.info(f"Auto-mapping joint states topic: {annotated_channel}")
                        try:
                            self.callback(annotated_channel)
                        except Exception as e:
                            logger.error(f"Error auto-mapping joint states topic: {e}")

class MessageConverter:
    """Handles conversion of LCM messages to JSON format for Foxglove"""
    def __init__(self, debug=False):
        self.debug = debug
        self.conversion_methods = {
            "sensor_msgs.image": self._format_image_msg,
            "sensor_msgs.compressedimage": self._format_compressed_image_msg,
            "tf2_msgs.tfmessage": self._format_tf_msg,
            "sensor_msgs.jointstate": self._format_joint_state_msg,
            # Fix case sensitivity for JointState message type
            "sensor_msgs.JointState": self._format_joint_state_msg,
            "sensor_msgs.pointcloud2": self._format_pointcloud2_msg,
            "sensor_msgs.PointCloud2": self._format_pointcloud2_msg,
        }
        # Cache for previously converted messages
        self.conversion_cache = {}
        # Stats for conversion timing
        self.conversion_times = defaultdict(list)
        
    def convert_message(self, topic_info, msg):
        """Convert an LCM message to Foxglove format"""
        # Try with exact type first
        if topic_info.schema_type in self.conversion_methods:
            converter = self.conversion_methods[topic_info.schema_type]
        else:
            # Fall back to case-insensitive check
            schema_type_lower = topic_info.schema_type.lower()
            if schema_type_lower in self.conversion_methods:
                converter = self.conversion_methods[schema_type_lower]
            else:
                # Log when we don't have a specialized converter
                logger.debug(f"No specialized converter for {topic_info.schema_type}, using generic conversion")
                return self._lcm_to_dict(msg)
        
        # Convert using the specialized converter
        try:
            start_time = time.time()
            result = converter(msg, topic_info)
            elapsed = time.time() - start_time
            
            # Update stats
            schema_key = topic_info.schema_type.lower()  # Use lowercase for stats
            self.conversion_times[schema_key].append(elapsed)
            if len(self.conversion_times[schema_key]) > 100:
                self.conversion_times[schema_key] = self.conversion_times[schema_key][-100:]
                
            # Log timing for slow conversions
            if elapsed > 0.1:  # More than 100ms
                avg_time = sum(self.conversion_times[schema_key]) / len(self.conversion_times[schema_key])
                logger.warning(f"Slow conversion for {topic_info.name}: {elapsed:.3f}s (avg: {avg_time:.3f}s)")
                
            return result
        except Exception as e:
            logger.error(f"Error in converter for {topic_info.schema_type}, falling back to generic: {e}")
            return self._lcm_to_dict(msg)

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
            # Return a minimal valid header
            now = time.time()
            return {
                "seq": 0,
                "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                "frame_id": ""
            }
        
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
                # Change the encoding string to rgb8
                encoding = "rgb8"
            
            if hasattr(msg, "data") and data_length > 0:
                # Get image data as bytes
                image_data_bytes = msg.data[:data_length]
                
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
            
            if hasattr(msg, "data") and data_length > 0:
                # Get image data as bytes
                image_data_bytes = msg.data[:data_length]
                
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
            return self._lcm_to_dict(msg)  # Fallback to generic conversion
    
    def _format_tf_msg(self, msg, topic_info=None):
        """Format a tf2_msgs/TFMessage for Foxglove"""
        try:
            # Get the transforms array with correct length
            transforms_length = getattr(msg, "transforms_length", 0)
            transforms = []
            
            # Process each transform in the array
            if hasattr(msg, "transforms") and transforms_length > 0:
                for i in range(min(transforms_length, len(msg.transforms))):
                    transform = msg.transforms[i]
                    transform_dict = self._format_transform_stamped(transform)
                    if transform_dict:
                        transforms.append(transform_dict)
            
            # Return properly formatted message
            return {"transforms": transforms}
            
        except Exception as e:
            logger.error(f"Error formatting TFMessage: {e}")
            return {"transforms": []}  # Return empty transforms array on error
    
    def _format_transform_stamped(self, transform):
        """Format a geometry_msgs/TransformStamped message for Foxglove"""
        try:
            # Format header
            header = self._get_header_dict(transform.header)
            
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
            return None
    
    def _format_joint_state_msg(self, msg, topic_info=None):
        """Format a sensor_msgs/JointState message for Foxglove"""
        try:
            # Debug log when processing a joint state
            logger.info(f"Processing JointState message from {topic_info.name if topic_info else 'unknown'}")
            
            # Format the header
            header = self._get_header_dict(msg.header)
            
            # Get array lengths and print debug info
            name_length = getattr(msg, "name_length", 0)
            position_length = getattr(msg, "position_length", 0)
            velocity_length = getattr(msg, "velocity_length", 0)
            effort_length = getattr(msg, "effort_length", 0)
            
            logger.info(f"JointState arrays: names={name_length}, positions={position_length}, velocities={velocity_length}, efforts={effort_length}")
            
            # Check if message has required attributes
            if not hasattr(msg, "name") or not hasattr(msg, "position"):
                logger.warning(f"JointState message missing required attributes name or position")
                # Log available attributes
                logger.warning(f"JointState available attributes: {[a for a in dir(msg) if not a.startswith('_')]}")
            
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
            
            # Log the conversion result
            logger.info(f"Converted JointState with {len(names)} joints: {names}")
            
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
            
            # Get basic properties
            data_len = len(msg.data) if hasattr(msg, 'data') else 0
            point_step = int(msg.point_step) if hasattr(msg, 'point_step') else 16
            
            # Basic validation check to avoid the "not a multiple of point_step" error
            if data_len % point_step != 0:
                logger.warning(f"PointCloud2 data length {data_len} is not a multiple of point_step {point_step}!")
                # Adjust the data to make it a multiple of point_step
                missing_bytes = point_step - (data_len % point_step)
                padded_data = msg.data + bytes(missing_bytes)
                msg.data = padded_data
                data_len = len(msg.data)
                
                # Also adjust width to match the new number of points
                adjusted_points = data_len // point_step
                msg.width = adjusted_points
            
            # Check if this is a colored point cloud by scanning field names
            has_rgb_fields = False
            has_rgba_field = False
            rgb_field_names = {"r", "g", "b"}
            
            # Extract field definitions
            if hasattr(msg, "fields") and fields_length > 0:
                for i in range(min(fields_length, len(msg.fields))):
                    field = msg.fields[i]
                    field_name = field.name.decode('utf-8', errors='replace') if isinstance(field.name, bytes) else field.name
                    
                    field_dict = {
                        "name": field_name,
                        "offset": int(field.offset),
                        "datatype": int(field.datatype),
                        "count": int(field.count)
                    }
                    fields.append(field_dict)
                    
                    # Check for RGB fields or RGBA packed field
                    if field_name.lower() in rgb_field_names:
                        has_rgb_fields = True
                    elif field_name.lower() == "rgba":
                        has_rgba_field = True
                
                # Handle based on color field type
                if has_rgba_field:
                    # We have a packed RGBA field - this is the preferred format for Foxglove
                    logger.info("Processing colored PointCloud2 with packed RGBA field")
                    
                    # Make sure fields are properly defined
                    if not all(any(f["name"].lower() == name for f in fields) for name in ["x", "y", "z", "rgba"]):
                        logger.warning("RGBA PointCloud2 is missing some required fields, adding defaults")
                        
                        # Add any missing required fields with default values
                        for i, (name, offset, datatype) in enumerate([
                            ("x", 0, 7),  # float32
                            ("y", 4, 7),  # float32
                            ("z", 8, 7),  # float32
                            ("rgba", 12, 6), # uint32
                        ]):
                            if not any(f["name"].lower() == name for f in fields):
                                fields.append({"name": name, "offset": offset, "datatype": datatype, "count": 1})
                elif has_rgb_fields:
                    # We have separate R,G,B fields
                    # Check if we have all the expected field definitions for colored points
                    if not all(any(f["name"].lower() == name for f in fields) for name in ["x", "y", "z", "r", "g", "b"]):
                        logger.warning("Colored PointCloud2 is missing some required fields, adding defaults")
                        
                        # Add any missing required fields with default values
                        for i, (name, offset, datatype) in enumerate([
                            ("x", 0, 7),  # float32
                            ("y", 4, 7),  # float32
                            ("z", 8, 7),  # float32
                            ("r", 12, 2), # uint8
                            ("g", 13, 2), # uint8
                            ("b", 14, 2), # uint8
                            ("a", 15, 2)  # uint8 (for alignment)
                        ]):
                            if not any(f["name"].lower() == name for f in fields):
                                fields.append({"name": name, "offset": offset, "datatype": datatype, "count": 1})
                    
                    logger.info("Processing colored PointCloud2 with RGB fields")
                else:
                    # No RGB fields, ensure we have intensity field for standard point cloud
                    if not any(f["name"].lower() == "intensity" for f in fields):
                        fields.append({"name": "intensity", "offset": 12, "datatype": 7, "count": 1})
            else:
                # If no fields, check if point step suggests RGB format (16 bytes = xyz+rgb+unused)
                if point_step == 16:
                    # This might be a colored point cloud, determine format based on data analysis
                    # First try with RGBA packed (better for Foxglove)
                    fields = [
                        {"name": "x", "offset": 0, "datatype": 7, "count": 1},  # Float32
                        {"name": "y", "offset": 4, "datatype": 7, "count": 1},  # Float32
                        {"name": "z", "offset": 8, "datatype": 7, "count": 1},  # Float32
                        {"name": "rgba", "offset": 12, "datatype": 6, "count": 1}  # UInt32
                    ]
                    has_rgba_field = True
                    logger.info("Detected possible colored PointCloud2, using XYZRGBA packed format")
                else:
                    # Default to standard XYZ + intensity format
                    fields = [
                        {"name": "x", "offset": 0, "datatype": 7, "count": 1},  # Float32
                        {"name": "y", "offset": 4, "datatype": 7, "count": 1},  # Float32
                        {"name": "z", "offset": 8, "datatype": 7, "count": 1},  # Float32
                        {"name": "intensity", "offset": 12, "datatype": 7, "count": 1}  # Float32
                    ]
            
            # Ensure we have at least x, y, z fields
            if len(fields) < 3:
                logger.warning(f"PointCloud2 has only {len(fields)} fields, adding x, y, z fields")
                for i, name in enumerate(["x", "y", "z"]):
                    if not any(f["name"].lower() == name for f in fields):
                        fields.append({"name": name, "offset": i*4, "datatype": 7, "count": 1})
            
            # Convert data to base64
            if hasattr(msg, "data") and len(msg.data) > 0:
                # Get point cloud data as bytes
                cloud_data_bytes = msg.data
                
                # Convert to base64
                cloud_data = base64.b64encode(cloud_data_bytes).decode("ascii")
            else:
                # Create a small point cloud with a single point at origin
                import struct
                if has_rgba_field:
                    # Create a colored point with packed RGBA (red)
                    # RGBA = (R << 24) | (G << 16) | (B << 8) | A
                    red_rgba = (255 << 24) | (0 << 16) | (0 << 8) | 255  # Red with alpha=255
                    cloud_data_bytes = struct.pack("<fffi", 0, 0, 0, red_rgba)  # XYZ+RGBA (red point)
                elif has_rgb_fields:
                    # Create a colored point with separate RGB components (red)
                    cloud_data_bytes = struct.pack("<fffBBBB", 0, 0, 0, 255, 0, 0, 255)  # XYZRGBA (red point)
                else:
                    # Create a standard point with intensity
                    cloud_data_bytes = struct.pack("<ffff", 0, 0, 0, 1.0)  # XYZ+intensity
                
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
            return self._lcm_to_dict(msg)  # Fallback to generic conversion

class MessageProcessor:
    """Processes LCM messages for conversion and sending to Foxglove"""
    def __init__(self, message_queue, server_queue, num_threads=DEFAULT_THREAD_POOL_SIZE, debug=False):
        self.message_queue = message_queue
        self.server_queue = server_queue
        self.converter = MessageConverter(debug=debug)
        self.running = True
        self.executor = ThreadPoolExecutor(max_workers=num_threads, thread_name_prefix="msg_processor")
        self.debug = debug
        self.message_cache = {}  # Cache for message hashes

    def start(self):
        """Start the message processor"""
        self.running = True
        self.executor.submit(self._process_messages_loop)
        logger.info(f"Message processor started with {self.executor._max_workers} threads")

    def stop(self):
        """Stop the message processor"""
        self.running = False
        self.executor.shutdown(wait=False)
        logger.info("Message processor stopped")

    def _process_messages_loop(self):
        """Main loop for processing messages"""
        while self.running:
            try:
                # Get a batch of messages to process
                messages = []
                message_count = 0
                
                try:
                    # Get the first message (blocking with timeout)
                    message = self.message_queue.get(block=True, timeout=0.1)
                    messages.append(message)
                    message_count += 1
                    
                    # Try to get more messages up to batch size (non-blocking)
                    for _ in range(MESSAGE_BATCH_SIZE - 1):
                        try:
                            message = self.message_queue.get(block=False)
                            messages.append(message)
                            message_count += 1
                        except queue.Empty:
                            break
                except queue.Empty:
                    # No messages to process
                    continue
                
                if not messages:
                    continue
                
                try:
                    # Group messages by topic
                    grouped_messages = defaultdict(list)
                    for msg in messages:
                        grouped_messages[msg.topic_info.full_topic_name].append(msg)
                    
                    # Process each topic's messages in parallel
                    futures = []
                    messages_to_process = []
                    
                    for topic_name, topic_messages in grouped_messages.items():
                        # Sort by priority
                        topic_messages.sort(key=lambda m: m.priority, reverse=True)
                        
                        # Only process the latest message for each topic if we have too many
                        if len(topic_messages) > 5:
                            to_process = [topic_messages[0]]  # Process the highest priority message
                            messages_to_process.extend(to_process)
                            # We'll mark the rest as done later
                            logger.debug(f"Skipping {len(topic_messages)-1} old messages for {topic_name}")
                        else:
                            messages_to_process.extend(topic_messages)
                    
                    # Submit all messages for processing
                    for msg in messages_to_process:
                        futures.append(self.executor.submit(self._process_single_message, msg))
                    
                    # Wait for all processing to complete
                    for future in futures:
                        try:
                            future.result()
                        except Exception as e:
                            logger.error(f"Error in message processing: {e}")
                    
                finally:
                    # Mark all fetched messages as done, regardless of processing success
                    # This ensures we don't call task_done too many times
                    for _ in range(message_count):
                        try:
                            self.message_queue.task_done()
                        except ValueError:
                            # If we already called task_done somewhere else, ignore the error
                            pass
                
            except Exception as e:
                logger.error(f"Error in message processor loop: {e}")
    
    def _process_single_message(self, lcm_message):
        """Process a single LCM message"""
        try:
            topic_info = lcm_message.topic_info
            data = lcm_message.data
            
            # Skip throttled topics
            now = time.time()
            min_interval_sec = topic_info.rate_limit_ms / 1000.0
            if min_interval_sec > 0 and now - topic_info.last_sent_timestamp < min_interval_sec:
                if self.debug:
                    logger.debug(f"Throttling message on {topic_info.name}")
                return
            
            # Skip if channel isn't registered yet
            if topic_info.channel_id is None:
                if self.debug:
                    logger.debug(f"Channel not registered for {topic_info.name}")
                return
                
            # Try to decode and convert the message
            if topic_info.lcm_class:
                try:
                    msg = topic_info.lcm_class.decode(data)
                    
                    # Check message caching
                    should_process = True
                    if topic_info.is_high_frequency:
                        # For high-frequency topics, check if the message is different from the last one
                        msg_hash = hash(data)
                        if topic_info.cache_hash == msg_hash:
                            should_process = False
                        else:
                            topic_info.cache_hash = msg_hash
                    
                    if should_process:
                        # Convert the message to dict format for Foxglove
                        msg_dict = self.converter.convert_message(topic_info, msg)
                        
                        # Update topic info
                        topic_info.last_sent_timestamp = now
                        topic_info.message_count += 1
                        
                        # Detect high-frequency topics
                        if topic_info.message_count >= 100 and not topic_info.is_high_frequency:
                            elapsed = now - lcm_message.receive_time
                            if elapsed < 5.0:  # 100 messages in less than 5 seconds
                                # This is a high-frequency topic
                                topic_info.is_high_frequency = True
                                # Set rate limit for high-frequency topics (100ms)
                                if topic_info.rate_limit_ms == 0:
                                    topic_info.rate_limit_ms = 100
                                logger.info(f"Detected high-frequency topic {topic_info.name}, rate limiting to {topic_info.rate_limit_ms}ms")
                        
                        # Put on server queue for sending
                        try:
                            timestamp_ns = int(now * 1e9)
                            self.server_queue.put((topic_info, msg_dict, timestamp_ns), block=False)
                        except queue.Full:
                            logger.debug(f"Server queue full, dropping message for {topic_info.name}")
                    
                except Exception as e:
                    logger.error(f"Error processing message for {topic_info.name}: {e}")
            
        except Exception as e:
            logger.error(f"Error in _process_single_message: {e}")
            # We no longer call task_done() here - it's handled in the main loop

class LcmFoxgloveBridgeRunner:
    """Runner class to manage the bridge and server lifecycle"""
    def __init__(self, host="0.0.0.0", port=8765, schema_map=None, debug=False, num_threads=DEFAULT_THREAD_POOL_SIZE):
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
        self.num_threads = num_threads
        
        # For cross-thread communication
        self.topic_queue = asyncio.Queue()
        self.message_queue = queue.PriorityQueue(maxsize=MAX_QUEUE_SIZE)
        self.server_queue = queue.Queue(maxsize=MAX_QUEUE_SIZE)
        self.loop = None
        self.server = None
        
        # For parallelization
        self.message_processor = None
        
        # Configure verbose debugging
        if debug:
            logger.setLevel(logging.DEBUG)
        
    async def run(self):
        """Run the bridge with proper context management for FoxgloveServer"""
        logger.info(f"Starting LCM-Foxglove bridge on {self.host}:{self.port} with {self.num_threads} threads")
        
        # Store reference to the event loop that will be used for all async operations
        self.loop = asyncio.get_running_loop()
        
        # Start the message processor
        self.message_processor = MessageProcessor(
            self.message_queue, 
            self.server_queue, 
            num_threads=self.num_threads,
            debug=self.debug
        )
        self.message_processor.start()
        
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
            
            # Start task to process messages for sending to server
            server_processor_task = asyncio.create_task(self._process_server_queue())
            
            try:
                # Keep running until interrupted
                while self.running:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                logger.info("\nTask cancelled")
            finally:
                # Cancel the processor tasks
                topic_processor_task.cancel()
                server_processor_task.cancel()
                try:
                    await topic_processor_task
                    await server_processor_task
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
                        
                        # Special handling for different message types
                        schema_type_lower = topic_info.schema_type.lower()
                        if schema_type_lower == "tf2_msgs.tfmessage":
                            logger.info(f"TF topic registered with channel ID: {channel_id}")
                            # Prioritize TF messages
                            topic_info.priority = 10
                        elif schema_type_lower == "sensor_msgs.pointcloud2":
                            logger.info(f"PointCloud2 topic registered with channel ID: {channel_id}")
                            # Rate limit PointCloud2 messages (they're large)
                            topic_info.rate_limit_ms = 100  # 10 Hz max
                        elif schema_type_lower in ["sensor_msgs.image", "sensor_msgs.compressedimage"]:
                            # Rate limit image messages
                            topic_info.rate_limit_ms = 50  # 20 Hz max
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
    
    async def _process_server_queue(self):
        """Process messages from the queue and send to Foxglove server"""
        while True:
            try:
                # Collect a batch of messages from the queue
                batched_messages = []
                
                # Try to get a message (blocking with timeout)
                try:
                    item = self.server_queue.get(block=True, timeout=0.01)
                    batched_messages.append(item)
                    
                    # Try to get more messages non-blocking
                    for _ in range(MESSAGE_BATCH_SIZE - 1):
                        try:
                            item = self.server_queue.get(block=False)
                            batched_messages.append(item)
                        except queue.Empty:
                            break
                except queue.Empty:
                    # No messages, just wait and try again
                    await asyncio.sleep(0.01)
                    continue
                
                # Process all collected messages
                for topic_info, msg_dict, timestamp_ns in batched_messages:
                    try:
                        if self.server and topic_info and topic_info.channel_id is not None:
                            # Convert the message to JSON
                            json_data = json.dumps(msg_dict).encode("utf-8")
                            
                            # Send to Foxglove
                            await self.server.send_message(topic_info.channel_id, timestamp_ns, json_data)
                            
                            # Log important messages being sent
                            schema_type_lower = topic_info.schema_type.lower()
                            if schema_type_lower in ["tf2_msgs.tfmessage", "sensor_msgs.pointcloud2"] and self.debug:
                                logger.debug(f"Sent {schema_type_lower} message on channel {topic_info.name}")
                        
                        # Mark as done
                        self.server_queue.task_done()
                    except Exception as e:
                        logger.error(f"Error sending message for {topic_info.name}: {e}")
                        # Make sure to mark as done even if there's an error
                        self.server_queue.task_done()
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in server processor: {e}")
    
    def _lcm_thread_func(self):
        """Thread for handling LCM messages"""
        while self.running:
            try:
                # Handle LCM messages until interrupted
                self.lc.handle_timeout(100)  # 100ms timeout
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
            
            # Set message priority based on message type
            schema_type_lower = schema_type.lower()
            if schema_type_lower == "tf2_msgs.tfmessage":
                topic_info.priority = 10  # Highest priority for TF
            elif schema_type_lower in ["sensor_msgs.pointcloud2"]:
                topic_info.priority = 5   # High priority for point clouds
            
            # Add topic to our map
            self.topics[topic_name] = topic_info
            
            # Queue the topic for registration with Foxglove
            if self.loop:
                self.loop.call_soon_threadsafe(self.topic_queue.put_nowait, topic_info)
            
            # Subscribe to the FULL LCM topic name (including schema annotation)
            subscription = self.lc.subscribe(topic_name, self._on_lcm_message)
            self.message_handlers[topic_name] = subscription
            
            logger.info(f"Subscribed to LCM topic: {topic_name}")
            
        except Exception as e:
            logger.error(f"Error processing topic {topic_name}: {e}")
            import traceback
            traceback.print_exc()
    
    def _on_lcm_message(self, channel, data):
        """Called when an LCM message is received"""
        # Get topic info
        topic_info = self.topics.get(channel)
        
        if not topic_info:
            if self.debug:
                logger.warning(f"Received message for unknown channel: {channel}")
            return
        
        try:
            # Calculate priority based on message type (higher = more important)
            priority = 0
            schema_type_lower = topic_info.schema_type.lower()
            if schema_type_lower == "tf2_msgs.tfmessage":
                priority = 10  # Highest priority for TF
            elif schema_type_lower in ["sensor_msgs.pointcloud2"]:
                priority = 5   # High priority for point clouds
            
            # Create a message container
            msg = LcmMessage(
                topic_info=topic_info,
                data=data,
                receive_time=time.time(),
                priority=priority
            )
            
            # Skip rate-limited topics early to avoid filling the queue
            min_interval_sec = topic_info.rate_limit_ms / 1000.0
            if (min_interval_sec > 0 and 
                topic_info.is_high_frequency and 
                time.time() - topic_info.last_sent_timestamp < min_interval_sec):
                # Too frequent, skip this message
                if self.debug:
                    logger.debug(f"Rate limiting message for {channel}")
                return
                
            # Skip if not yet registered with Foxglove
            if topic_info.channel_id is None:
                if self.debug:
                    logger.debug(f"Channel not yet registered for {channel}, skipping message")
                return
            
            # Push to queue for processing by the thread pool
            try:
                self.message_queue.put(msg, block=False)
            except queue.Full:
                # Queue is full, drop this message
                if self.debug:
                    logger.warning(f"Message queue full, dropping message for {channel}")
        except Exception as e:
            logger.error(f"Error queuing message on {channel}: {e}")
            
    def stop(self):
        """Stop the bridge cleanly"""
        logger.info("Stopping LCM-Foxglove bridge")
        self.running = False
        if self.discoverer:
            self.discoverer.stop()
        if self.message_processor:
            self.message_processor.stop()

async def main():
    """Main entry point"""
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='LCM to Foxglove WebSocket Bridge')
    parser.add_argument('--host', default='0.0.0.0', help='WebSocket server host')
    parser.add_argument('--port', type=int, default=8765, help='WebSocket server port')
    parser.add_argument('--debug', action='store_true', help='Enable verbose debug output')
    parser.add_argument('--map-file', type=str, help='JSON file mapping topic names to schema types')
    parser.add_argument('--threads', type=int, default=DEFAULT_THREAD_POOL_SIZE, 
                        help=f'Number of threads for message processing (default: {DEFAULT_THREAD_POOL_SIZE})')
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
        debug=args.debug,
        num_threads=args.threads
    )
    await bridge.run()

if __name__ == "__main__":
    # Add python_lcm_msgs to path so we can import LCM message modules
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lcm_module_dir = os.path.join(current_dir, "python_lcm_msgs")
    sys.path.append(lcm_module_dir)
    
    # Run the main function
    run_cancellable(main())
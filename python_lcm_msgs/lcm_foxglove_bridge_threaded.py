#!/usr/bin/env python3
"""
LCM to Foxglove Bridge (Multi-Threaded)

This script bridges LCM (Lightweight Communications and Marshalling) messages to Foxglove Studio.
It subscribes to specified LCM topics and republishes them as Foxglove messages in real-time,
using separate threads for each topic to improve performance.

Usage:
    python lcm_foxglove_bridge_threaded.py [options]

Options:
    --lcm-url TEXT      LCM URL to subscribe to [default: udpm://239.255.76.67:7667]
    --foxglove-port INT Port for Foxglove server [default: 8765]
    --topic-map PATH    Path to JSON file mapping LCM topics to Foxglove schemas
    --topic TOPIC       LCM topic:foxglove_schema_type mapping (can be used multiple times)
    --verbose           Enable verbose logging
    --help              Show this message and exit
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
import queue
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Optional, Set, Tuple, Callable

import lcm
import foxglove
from foxglove import Channel, Schema
from foxglove.websocket import (
    Capability,
    ServerListener,
)
from foxglove.channels import (
    ColorChannel,
    FrameTransformChannel,
    FrameTransformsChannel,
    PackedElementFieldChannel,
    PointCloudChannel,
    PoseChannel,
    QuaternionChannel,
    RawImageChannel,
    SceneEntityChannel,
    SceneUpdateChannel,
    Vector3Channel,
)
from foxglove.schemas import (
    Color,
    CubePrimitive,
    Duration,
    FrameTransform,
    FrameTransforms,
    PackedElementField,
    PackedElementFieldNumericType,
    PointCloud,
    Pose,
    Quaternion,
    RawImage,
    SceneEntity,
    SceneUpdate,
    Timestamp,
    Vector3,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('lcm_foxglove_bridge')

class MessageProcessor:
    """
    Handles processing of a single LCM topic with its own thread.
    """
    
    def __init__(
        self,
        channel: str,
        schema_type: str,
        converter_func: Callable,
        foxglove_channel: Any,
        max_queue_size: int = 100
    ):
        """
        Initialize a message processor for a specific LCM topic.
        
        Args:
            channel: LCM channel name
            schema_type: Foxglove schema type
            converter_func: Function to convert LCM messages to Foxglove format
            foxglove_channel: Foxglove channel for publishing
            max_queue_size: Maximum size of message queue
        """
        self.channel = channel
        self.schema_type = schema_type
        self.converter_func = converter_func
        self.foxglove_channel = foxglove_channel
        self.message_queue = queue.Queue(maxsize=max_queue_size)
        self.thread = None
        self.stop_requested = False
        self.messages_processed = 0
        self.messages_dropped = 0
        self.processing_time = 0.0
        self.last_stats_time = time.time()
    
    def enqueue_message(self, data: bytes) -> bool:
        """
        Add a message to the processing queue.
        
        Args:
            data: Raw LCM message data
            
        Returns:
            True if message was added to queue, False if queue was full
        """
        try:
            self.message_queue.put_nowait(data)
            return True
        except queue.Full:
            self.messages_dropped += 1
            return False
    
    def _process_messages(self):
        """Process messages from the queue until stopped."""
        logger.info(f"Started message processor for channel: {self.channel}")
        
        while not self.stop_requested:
            try:
                # Get message with timeout to allow for periodic checks of stop_requested
                try:
                    data = self.message_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # Process the message
                start_time = time.time()
                try:
                    # Convert message
                    converted_msg = self.converter_func(self.channel, data)
                    
                    # Publish to Foxglove if conversion was successful
                    if converted_msg is not None:
                        self.foxglove_channel.log(converted_msg)
                        logger.debug(f"Published message to Foxglove for {self.channel}")
                    
                    self.messages_processed += 1
                    self.processing_time += (time.time() - start_time)
                    
                    # Log performance stats periodically
                    current_time = time.time()
                    if current_time - self.last_stats_time > 10.0:  # Every 10 seconds
                        self._log_stats()
                        self.last_stats_time = current_time
                        
                except Exception as e:
                    logger.error(f"Error processing message for {self.channel}: {e}")
                    import traceback
                    logger.debug(traceback.format_exc())
                
                # Mark task as done
                self.message_queue.task_done()
                
            except Exception as e:
                logger.error(f"Unexpected error in message processor for {self.channel}: {e}")
        
        logger.info(f"Stopped message processor for channel: {self.channel}")
    
    def _log_stats(self):
        """Log performance statistics."""
        avg_processing_time = (self.processing_time / max(1, self.messages_processed)) * 1000  # ms
        
        logger.info(f"Channel {self.channel} stats: "
                   f"processed={self.messages_processed}, "
                   f"dropped={self.messages_dropped}, "
                   f"avg_time={avg_processing_time:.2f}ms, "
                   f"queue_size={self.message_queue.qsize()}")
        
        # Reset counters
        self.messages_processed = 0
        self.messages_dropped = 0
        self.processing_time = 0.0
    
    def start(self):
        """Start the message processor thread."""
        if self.thread is not None and self.thread.is_alive():
            return  # Already running
        
        self.stop_requested = False
        self.thread = threading.Thread(target=self._process_messages)
        self.thread.daemon = True
        self.thread.start()
    
    def stop(self):
        """Stop the message processor thread."""
        self.stop_requested = True
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                logger.warning(f"Message processor thread for {self.channel} did not stop cleanly")


class LCMFoxgloveBridge:
    """
    Multi-threaded bridge for converting LCM messages to Foxglove Studio format and publishing them.
    """
    
    def __init__(
        self, 
        lcm_url: str = "udpm://239.255.76.67:7667", 
        foxglove_port: int = 8765,
        topic_map: Dict[str, str] = None
    ):
        """
        Initialize the bridge.
        
        Args:
            lcm_url: LCM URL to subscribe to
            foxglove_port: Port for Foxglove server
            topic_map: Mapping from LCM topics to Foxglove schema types
        """
        self.lcm_url = lcm_url
        self.foxglove_port = foxglove_port
        self.topic_map = topic_map or {}
        
        # Internal state
        self.lc = None
        self.server = None
        self.subscriptions = []
        self.channels = {}
        self.stop_requested = False
        self.lcm_thread = None
        self.processors = {}  # Maps channels to MessageProcessor instances
        
        # Type conversion mappings - maps schema type names to converter functions
        self.type_converters = {
            "foxglove.FrameTransform": self._convert_transform,
            "foxglove.FrameTransforms": self._convert_transforms,
            "foxglove.PointCloud": self._convert_pointcloud,
            "foxglove.RawImage": self._convert_image,
            "foxglove.CompressedImage": self._convert_compressed_image,
            "foxglove.SceneEntity": self._convert_scene_entity,
            "foxglove.SceneUpdate": self._convert_scene_update,
            "foxglove.PoseInFrame": self._convert_pose,
            "foxglove.LaserScan": self._convert_laserscan,
            "foxglove.LocationFix": self._convert_location,
            "foxglove.Log": self._convert_log,
            "foxglove.Vector3": self._convert_vector3,
            # Special case for generic messages
            "json": self._convert_json,
        }
        
        # Auto-detect mapping
        self.auto_detect_mapping = {
            "transform": self._convert_transform,
            "tf": self._convert_transform,
            "point_cloud": self._convert_pointcloud,
            "pointcloud": self._convert_pointcloud,
            "image": self._convert_image,
            "camera": self._convert_image,
            "compressed_image": self._convert_compressed_image,
            "laser": self._convert_laserscan,
            "scan": self._convert_laserscan,
            "lidar": self._convert_laserscan,
            "pose": self._convert_pose,
            "location": self._convert_location,
            "gps": self._convert_location,
            "fix": self._convert_location,
            "log": self._convert_log,
            "vector3": self._convert_vector3,
        }
        
        # Initialize LCM and Foxglove server
        self._init_lcm()
        self._init_foxglove()
        
    def _init_lcm(self):
        """Initialize LCM connection."""
        try:
            self.lc = lcm.LCM(self.lcm_url)
            logger.info(f"Connected to LCM at {self.lcm_url}")
        except Exception as e:
            logger.error(f"Failed to connect to LCM: {e}")
            raise
    
    def _init_foxglove(self):
        """Initialize Foxglove server."""
        try:
            # Create server with a custom listener
            self.server = foxglove.start_server(
                server_listener=LCMFoxgloveListener(),
                capabilities=[Capability.ClientPublish],
                supported_encodings=["json"],
            )
            logger.info(f"Started Foxglove server on port {self.foxglove_port}")
        except Exception as e:
            logger.error(f"Failed to start Foxglove server: {e}")
            raise
    
    def _setup_channels(self):
        """Set up Foxglove channels based on topic map."""
        for lcm_topic, schema_type in self.topic_map.items():
            try:
                # Create channel based on schema type
                if schema_type in ["json", "auto"]:
                    # For JSON channels, we don't need a specific schema
                    self.channels[lcm_topic] = Channel(topic=f"/{lcm_topic}")
                    logger.info(f"Created generic JSON channel for LCM topic: {lcm_topic}")
                else:
                    # For typed channels, use the specific schema
                    if schema_type == "foxglove.Color":
                        self.channels[lcm_topic] = ColorChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.FrameTransform":
                        self.channels[lcm_topic] = FrameTransformChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.FrameTransforms":
                        self.channels[lcm_topic] = FrameTransformsChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.PackedElementField":
                        self.channels[lcm_topic] = PackedElementFieldChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.PointCloud":
                        self.channels[lcm_topic] = PointCloudChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.Pose":
                        self.channels[lcm_topic] = PoseChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.Quaternion":
                        self.channels[lcm_topic] = QuaternionChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.RawImage":
                        self.channels[lcm_topic] = RawImageChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.SceneEntity":
                        self.channels[lcm_topic] = SceneEntityChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.SceneUpdate":
                        self.channels[lcm_topic] = SceneUpdateChannel(topic=f"/{lcm_topic}")
                    elif schema_type == "foxglove.Vector3":
                        self.channels[lcm_topic] = Vector3Channel(topic=f"/{lcm_topic}")
                    logger.info(f"Created {schema_type} channel for LCM topic: {lcm_topic}")
            except Exception as e:
                logger.error(f"Failed to create channel for {lcm_topic}: {e}")
    
    def _setup_processors(self):
        """Set up message processors for each topic."""
        for lcm_topic, schema_type in self.topic_map.items():
            if lcm_topic not in self.channels:
                logger.warning(f"No channel found for {lcm_topic}, skipping processor setup")
                continue
                
            try:
                # Determine converter function based on schema type
                if schema_type == "auto":
                    converter = self._auto_detect_converter
                else:
                    converter = self.type_converters.get(schema_type, self._convert_json)
                
                # Create processor
                processor = MessageProcessor(
                    channel=lcm_topic,
                    schema_type=schema_type,
                    converter_func=converter,
                    foxglove_channel=self.channels[lcm_topic]
                )
                
                self.processors[lcm_topic] = processor
                logger.info(f"Created message processor for {lcm_topic} with schema type {schema_type}")
                
            except Exception as e:
                logger.error(f"Failed to create processor for {lcm_topic}: {e}")
    
    def _subscribe_to_lcm_topics(self):
        """Subscribe to LCM topics."""
        for lcm_topic in self.topic_map:
            try:
                subscription = self.lc.subscribe(lcm_topic, self._on_lcm_message)
                self.subscriptions.append(subscription)
                logger.info(f"Subscribed to LCM topic: {lcm_topic}")
            except Exception as e:
                logger.error(f"Failed to subscribe to {lcm_topic}: {e}")
    
    def _on_lcm_message(self, channel, data):
        """
        Callback when an LCM message is received.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
        """
        if self.stop_requested:
            return
        
        # Get processor for this channel
        processor = self.processors.get(channel)
        if processor:
            # Enqueue message for processing
            if not processor.enqueue_message(data):
                logger.warning(f"Message queue full for channel {channel}, dropping message")
        else:
            logger.warning(f"No processor found for channel {channel}")
    
    def _auto_detect_converter(self, channel: str, data: bytes) -> Any:
        """
        Auto-detect message type and convert accordingly.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Converted message in Foxglove format
        """
        # Implement simple heuristics based on channel name
        channel_lower = channel.lower()
        
        for key, converter in self.auto_detect_mapping.items():
            if key in channel_lower:
                if key == "compressed_image" and "compressed" in channel_lower:
                    return converter(channel, data)
                elif key == "image" and "compressed" not in channel_lower:
                    return converter(channel, data)
                elif key != "image" and key != "compressed_image":
                    return converter(channel, data)
        
        # Default to JSON for unknown types
        return self._convert_json(channel, data)
    

    def _convert_vector3(self, channel: str, data: bytes) -> Dict:
        """
        Convert LCM Vector3 message to JSON format for generic Foxglove channels.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove Vector3
        """
        from geometry_msgs import Vector3 as GeometryVector3

        try:
            msg = GeometryVector3.decode(data)
            
            return Vector3(x=msg.x, y=msg.y, z=msg.z)
        except Exception as e:
            logger.error(f"Error converting vector3 message: {e}")
            return None
    
    def _convert_transform(self, channel: str, data: bytes) -> FrameTransform:
        """
        Convert LCM transform message to Foxglove FrameTransform.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove FrameTransform
        """
        try:
            from geometry_msgs import TransformStamped
            
            # If the channel is "tf", we're likely receiving a TFMessage
            # with multiple transforms, so handle it appropriately
            if channel == "tf":
                # This is probably a TFMessage, let's check the first transform
                from tf2_msgs import TFMessage as TF2Message
                
                # Try to decode as a TF2Message
                try:
                    tf_msg = TF2Message.decode(data)
                    
                    # If there are any transforms, return the first one
                    if tf_msg.transforms_length > 0:
                        logger.debug(f"Converting first of {tf_msg.transforms_length} transforms from TFMessage")
                        # Convert the first transform
                        return self._convert_transform_stamped(tf_msg.transforms[0])
                    else:
                        logger.warning("Received empty TFMessage with no transforms")
                        return None
                    
                except Exception as e:
                    logger.error(f"Error decoding TFMessage: {e}")
                    # Fall through to try decoding as a single TransformStamped
            
            # Try to decode as a TransformStamped directly
            try:
                transform_stamped = TransformStamped.decode(data)
                return self._convert_transform_stamped(transform_stamped)
            except Exception as e:
                logger.error(f"Error decoding TransformStamped: {e}")
                return None
                
        except Exception as e:
            logger.error(f"Error in transform conversion: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
            
    def _convert_transform_stamped(self, transform_stamped) -> FrameTransform:
        """
        Convert a LCM TransformStamped to a Foxglove FrameTransform.
        
        Args:
            transform_stamped: The LCM TransformStamped message
            
        Returns:
            A Foxglove FrameTransform message
        """
        try:
            # Extract timestamp from the header
            timestamp = Timestamp(
                sec=int(transform_stamped.header.stamp // 1_000_000_000) if transform_stamped.header.stamp > 1_000_000_000 else int(transform_stamped.header.stamp),
                nsec=int(transform_stamped.header.stamp % 1_000_000_000) if transform_stamped.header.stamp > 1_000_000_000 else 0
            )
            
            # Get parent and child frame IDs
            parent_frame = transform_stamped.header.frame_id
            child_frame = transform_stamped.child_frame_id
            
            # Create the translation and rotation
            translation = Vector3(
                x=transform_stamped.transform.translation.x,
                y=transform_stamped.transform.translation.y,
                z=transform_stamped.transform.translation.z
            )
            
            rotation = Quaternion(
                x=transform_stamped.transform.rotation.x,
                y=transform_stamped.transform.rotation.y,
                z=transform_stamped.transform.rotation.z,
                w=transform_stamped.transform.rotation.w
            )
            
            # Create Foxglove FrameTransform according to the API
            transform = FrameTransform(
                timestamp=timestamp,
                parent_frame_id=parent_frame,
                child_frame_id=child_frame,
                translation=translation,
                rotation=rotation
            )
            
            logger.debug(f"Converted transform from {parent_frame} to {child_frame}")
            return transform
            
        except Exception as e:
            logger.error(f"Error converting TransformStamped: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
    
    def _convert_transforms(self, channel: str, data: bytes) -> FrameTransforms:
        """
        Convert LCM transforms message to Foxglove FrameTransforms.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove FrameTransforms
        """
        try:
            # Parse the TFMessage
            from tf2_msgs import TFMessage as TF2Message
            
            # Decode the message
            tf_msg = TF2Message.decode(data)
            
            # Process all transforms in the message
            foxglove_transforms = []
            
            for transform_stamped in tf_msg.transforms:
                # Convert each TransformStamped to Foxglove format
                foxglove_transform = self._convert_transform_stamped(transform_stamped)
                if foxglove_transform:
                    foxglove_transforms.append(foxglove_transform)
            
            # Create Foxglove FrameTransforms
            if foxglove_transforms:
                logger.debug(f"Converted {len(foxglove_transforms)} transforms from TFMessage")
                
                # Create a FrameTransforms object with the list of transforms
                transforms = FrameTransforms(
                    transforms=foxglove_transforms
                )
                return transforms
            else:
                logger.warning("No valid transforms found in TFMessage")
                return None
                
        except Exception as e:
            logger.error(f"Error converting transforms message: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
    
    def _convert_pointcloud(self, channel: str, data: bytes) -> PointCloud:
        """
        Convert LCM point cloud message to Foxglove PointCloud.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove PointCloud
        """
        # Placeholder implementation
        try:
            logger.warning(f"Using dummy implementation for point cloud conversion on {channel}")
            
            # Create a simple point cloud with a few points
            timestamp = Timestamp.from_seconds(time.time())
            
            # Define fields for the point cloud
            f32 = PackedElementFieldNumericType.Float32
            u32 = PackedElementFieldNumericType.Uint32
            
            # Create a simple buffer with a few points
            import struct
            point_struct = struct.Struct("<fffBBBB")
            num_points = 10
            buffer = bytearray(point_struct.size * num_points)
            
            for i in range(num_points):
                x, y, z = i * 0.1, i * 0.2, i * 0.3
                r, g, b, a = 255, 0, 0, 255  # Red points
                point_struct.pack_into(buffer, i * point_struct.size, x, y, z, b, g, r, a)
            
            # Create Foxglove PointCloud
            pointcloud = PointCloud(
                timestamp=timestamp,
                frame_id="map",
                pose=Pose(
                    position=Vector3(x=0, y=0, z=0),
                    orientation=Quaternion(x=0, y=0, z=0, w=1),
                ),
                point_stride=16,  # 4 fields * 4 bytes
                fields=[
                    PackedElementField(name="x", offset=0, type=f32),
                    PackedElementField(name="y", offset=4, type=f32),
                    PackedElementField(name="z", offset=8, type=f32),
                    PackedElementField(name="rgba", offset=12, type=u32),
                ],
                data=bytes(buffer),
            )
            
            return pointcloud
        except Exception as e:
            logger.error(f"Error converting point cloud message: {e}")
            return None
    
    def _convert_image(self, channel: str, data: bytes) -> Dict:
        """
        Convert LCM image message to Foxglove RawImage format.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove RawImage
        """
        try:
            from sensor_msgs.Image import Image
            
            # Decode the LCM Image message
            lcm_image = Image.decode(data)
            
            # Calculate timestamp from header
            timestamp_sec = int(lcm_image.header.stamp // 1_000_000_000)
            timestamp_nsec = int(lcm_image.header.stamp % 1_000_000_000)
            
            # Create Foxglove's timestamp
            timestamp = Timestamp(sec=timestamp_sec, nsec=timestamp_nsec)
            
            # Create Foxglove RawImage using the foxglove.schemas.RawImage
            raw_image = RawImage(
                timestamp=timestamp,
                frame_id=lcm_image.header.frame_id,
                width=lcm_image.width,
                height=lcm_image.height,
                encoding=lcm_image.encoding,
                step=lcm_image.step,
                data=lcm_image.data
            )
            
            # Log the image conversion details using input message attributes
            logger.debug(f"Converted image from {channel}: dimensions={lcm_image.width}x{lcm_image.height}, encoding={lcm_image.encoding}")
            
            return raw_image
        except Exception as e:
            logger.error(f"Error converting image message: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
    
    def _convert_compressed_image(self, channel: str, data: bytes) -> Any:
        """
        Convert LCM compressed image message to Foxglove CompressedImage.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove CompressedImage
        """
        try:
            from sensor_msgs.CompressedImage import CompressedImage as RosCompressedImage
            from foxglove.schemas import CompressedImage
            
            # Decode the LCM CompressedImage message
            lcm_image = RosCompressedImage.decode(data)
            
            # Calculate timestamp from header
            timestamp_sec = int(lcm_image.header.stamp // 1_000_000_000)
            timestamp_nsec = int(lcm_image.header.stamp % 1_000_000_000)
            
            # Create Foxglove's timestamp
            timestamp = Timestamp(sec=timestamp_sec, nsec=timestamp_nsec)
            
            # Create Foxglove CompressedImage
            fox_image = CompressedImage(
                timestamp=timestamp,
                frame_id=lcm_image.header.frame_id,
                data=lcm_image.data,
                format=lcm_image.format
            )
            
            # CompressedImage in foxglove.schemas may have a different attribute structure
            logger.debug(f"Converted compressed image from {channel}: format={lcm_image.format}, data size={len(lcm_image.data)}")
            
            return fox_image
        except Exception as e:
            logger.error(f"Error converting compressed image message: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
    
    def _convert_scene_entity(self, channel: str, data: bytes) -> SceneEntity:
        """
        Convert LCM message to Foxglove SceneEntity.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove SceneEntity
        """
        try:
            logger.warning(f"Using dummy implementation for scene entity conversion on {channel}")
            
            # Create a simple cube entity
            timestamp = Timestamp.from_seconds(time.time())
            
            entity = SceneEntity(
                timestamp=timestamp,
                frame_id="map",
                id="cube1",
                lifetime=Duration.from_secs(0),  # Permanent
                frame_locked=False,
                cubes=[
                    CubePrimitive(
                        pose=Pose(
                            position=Vector3(x=1.0, y=1.0, z=1.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        ),
                        size=Vector3(x=0.5, y=0.5, z=0.5),
                        color=Color(r=1.0, g=0.0, b=0.0, a=1.0)  # Red cube
                    )
                ]
            )
            
            return entity
        except Exception as e:
            logger.error(f"Error converting scene entity message: {e}")
            return None
    
    def _convert_scene_update(self, channel: str, data: bytes) -> SceneUpdate:
        """
        Convert LCM message to Foxglove SceneUpdate.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove SceneUpdate
        """
        try:
            logger.warning(f"Using dummy implementation for scene update conversion on {channel}")
            
            # Create a scene update with a single entity
            entity = self._convert_scene_entity(channel, data)
            
            update = SceneUpdate(
                entities=[entity],
                deletions=[]
            )
            
            return update
        except Exception as e:
            logger.error(f"Error converting scene update message: {e}")
            return None
    
    def _convert_pose(self, channel: str, data: bytes) -> Any:
        """
        Convert LCM pose message to Foxglove PoseInFrame.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove PoseInFrame
        """
        try:
            from foxglove.schemas import PoseInFrame
            
            logger.warning(f"Using dummy implementation for pose conversion on {channel}")
            
            pose_in_frame = PoseInFrame(
                timestamp=Timestamp.from_seconds(time.time()),
                frame_id="map",
                pose=Pose(
                    position=Vector3(x=1.0, y=2.0, z=3.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
            
            return pose_in_frame
        except Exception as e:
            logger.error(f"Error converting pose message: {e}")
            return None
    
    def _convert_laserscan(self, channel: str, data: bytes) -> Any:
        """
        Convert LCM laser scan message to Foxglove LaserScan.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove LaserScan
        """
        try:
            from foxglove.schemas import LaserScan
            import numpy as np
            
            logger.warning(f"Using dummy implementation for laser scan conversion on {channel}")
            
            # Create a dummy laser scan with 360 points
            num_points = 360
            start_angle = -3.14159  # -pi radians
            end_angle = 3.14159   # pi radians
            
            # Generate ranges (from 1m to 10m)
            ranges = np.linspace(1.0, 10.0, num_points).tolist()
            intensities = np.ones(num_points).tolist()  # Constant intensity
            
            laser_scan = LaserScan(
                timestamp=Timestamp.from_seconds(time.time()),
                frame_id="laser",
                pose=Pose(
                    position=Vector3(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ),
                start_angle=start_angle,
                end_angle=end_angle,
                ranges=ranges,
                intensities=intensities
            )
            
            return laser_scan
        except Exception as e:
            logger.error(f"Error converting laser scan message: {e}")
            return None
    
    def _convert_location(self, channel: str, data: bytes) -> Any:
        """
        Convert LCM location message to Foxglove LocationFix.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove LocationFix
        """
        try:
            from foxglove.schemas import LocationFix, PositionCovarianceType
            
            logger.warning(f"Using dummy implementation for location conversion on {channel}")
            
            # Create a dummy location fix (San Francisco)
            location_fix = LocationFix(
                timestamp=Timestamp.from_seconds(time.time()),
                frame_id="gps",
                latitude=37.7749,
                longitude=-122.4194,
                altitude=10.0,
                position_covariance=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                position_covariance_type=PositionCovarianceType.APPROXIMATED
            )
            
            return location_fix
        except Exception as e:
            logger.error(f"Error converting location message: {e}")
            return None
    
    def _convert_log(self, channel: str, data: bytes) -> Any:
        """
        Convert LCM log message to Foxglove Log.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            Foxglove Log
        """
        try:
            from foxglove.schemas import Log, LogLevel
            
            logger.warning(f"Using dummy implementation for log conversion on {channel}")
            
            # Create a dummy log message
            log_msg = Log(
                timestamp=Timestamp.from_seconds(time.time()),
                level=LogLevel.INFO,
                message=f"LCM message received on {channel}",
                name="lcm_bridge",
                file="lcm_foxglove_bridge.py",
                line=0
            )
            
            return log_msg
        except Exception as e:
            logger.error(f"Error converting log message: {e}")
            return None
    
    def _convert_json(self, channel: str, data: bytes) -> Dict:
        """
        Convert LCM message to JSON format for generic Foxglove channels.
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
            
        Returns:
            JSON-compatible dictionary
        """
        try:
            # For generic JSON, we'll just create a simple wrapper
            # In a real implementation, you would use the appropriate LCM decode
            # function based on the message type
            
            # Placeholder implementation
            return {
                "timestamp": time.time(),
                "channel": channel,
                "data_size": len(data),
                "data_hex": data.hex()[:20] + "..." if len(data) > 10 else data.hex(),
                # Add parsed fields based on your LCM type
            }
        except Exception as e:
            logger.error(f"Error converting message to JSON: {e}")
            return None
    
    def start(self):
        """Start the bridge."""
        try:
            # Set up channels and processors
            self._setup_channels()
            self._setup_processors()
            
            # Start all processors
            for processor in self.processors.values():
                processor.start()
            
            # Subscribe to LCM topics
            self._subscribe_to_lcm_topics()
            
            # Start LCM handling thread
            self.stop_requested = False
            self.lcm_thread = threading.Thread(target=self._lcm_handler_loop)
            self.lcm_thread.daemon = True
            self.lcm_thread.start()
            
            logger.info("Bridge started successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to start bridge: {e}")
            self.stop()
            return False
    
    def _lcm_handler_loop(self):
        """Main loop for handling LCM messages."""
        logger.info("LCM handler thread started")
        try:
            while not self.stop_requested:
                # Handle LCM messages with timeout
                timeout = 100  # ms
                self.lc.handle_timeout(timeout)
        except Exception as e:
            logger.error(f"Error in LCM handler loop: {e}")
        finally:
            logger.info("LCM handler thread exiting")
    
    def stop(self):
        """Stop the bridge."""
        logger.info("Stopping bridge...")
        self.stop_requested = True
        
        # Stop all processors
        for processor in self.processors.values():
            processor.stop()
        
        # Wait for LCM thread to finish
        if self.lcm_thread and self.lcm_thread.is_alive():
            self.lcm_thread.join(timeout=2.0)
        
        # Close LCM
        self.lc = None
        
        # Stop Foxglove server
        if self.server:
            self.server.stop()
            self.server = None
        
        logger.info("Bridge stopped")


class LCMFoxgloveListener(ServerListener):
    """Custom listener for Foxglove server events."""
    
    def __init__(self):
        self.subscribers = {}
        
    def on_subscribe(self, client, channel):
        """Called when a client subscribes to a channel."""
        logger.info(f"Foxglove client {client.id} subscribed to channel {channel.topic}")
        self.subscribers.setdefault(client.id, set()).add(channel.topic)
        
    def on_unsubscribe(self, client, channel):
        """Called when a client unsubscribes from a channel."""
        logger.info(f"Foxglove client {client.id} unsubscribed from channel {channel.topic}")
        if client.id in self.subscribers:
            self.subscribers[client.id].remove(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def parse_topic_mapping(topic_map_str: str) -> Tuple[str, str]:
    """
    Parse a topic mapping string in the format "lcm_topic:foxglove_schema_type".
    
    Args:
        topic_map_str: Topic mapping string
    
    Returns:
        Tuple of (lcm_topic, foxglove_schema_type)
    """
    parts = topic_map_str.split(":", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid topic mapping format: {topic_map_str}. Expected format: lcm_topic:foxglove_schema_type")
    
    return parts[0], parts[1]


def load_topic_map_from_file(file_path: str) -> Dict[str, str]:
    """
    Load topic mapping from a JSON file.
    
    Args:
        file_path: Path to the JSON file
    
    Returns:
        Dictionary mapping from LCM topics to Foxglove schema types
    """
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading topic map from {file_path}: {e}")
        raise


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Multi-Threaded LCM to Foxglove Bridge')
    parser.add_argument('--lcm-url', default="udpm://239.255.76.67:7667",
                        help='LCM URL to subscribe to')
    parser.add_argument('--foxglove-port', type=int, default=8765,
                        help='Port for Foxglove server')
    parser.add_argument('--topic-map', type=str,
                        help='Path to JSON file mapping LCM topics to Foxglove schemas')
    parser.add_argument('--topic', action='append', type=str,
                        help='LCM topic:foxglove_schema_type mapping (can be used multiple times)')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Configure logging level
    if args.verbose:
        logger.setLevel(logging.DEBUG)
        foxglove.set_log_level(logging.DEBUG)
    
    # Load topic mapping
    topic_map = {}
    
    # Load from file if specified
    if args.topic_map:
        try:
            topic_map.update(load_topic_map_from_file(args.topic_map))
        except Exception as e:
            logger.error(f"Error loading topic map: {e}")
            return 1
    
    # Load from command line arguments
    if args.topic:
        for topic_str in args.topic:
            try:
                lcm_topic, schema_type = parse_topic_mapping(topic_str)
                topic_map[lcm_topic] = schema_type
            except ValueError as e:
                logger.error(str(e))
                return 1
    
    # Check if any topics are specified
    if not topic_map:
        logger.error("No topics specified. Use --topic-map or --topic to specify topics to bridge.")
        return 1
    
    # Create and start bridge
    bridge = LCMFoxgloveBridge(
        lcm_url=args.lcm_url,
        foxglove_port=args.foxglove_port,
        topic_map=topic_map
    )
    
    # Register signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Stopping due to signal...")
        bridge.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start bridge
    if not bridge.start():
        return 1
    
    # Main loop - keep process alive
    logger.info("Multi-Threaded Bridge running. Press Ctrl+C to exit.")
    logger.info(f"Bridging the following topics:")
    for lcm_topic, schema_type in topic_map.items():
        logger.info(f"  {lcm_topic} -> {schema_type}")
    
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        bridge.stop()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
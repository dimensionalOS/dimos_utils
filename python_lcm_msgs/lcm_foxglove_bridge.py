#!/usr/bin/env python3
"""
LCM to Foxglove Bridge

This script bridges LCM (Lightweight Communications and Marshalling) messages to Foxglove Studio.
It subscribes to specified LCM topics and republishes them as Foxglove messages in real-time.

Usage:
    python lcm_foxglove_bridge.py [options]

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

class LCMFoxgloveBridge:
    """
    Bridge for converting LCM messages to Foxglove Studio format and publishing them.
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
        
        # Get schema type for this channel
        schema_type = self.topic_map.get(channel)
        if not schema_type:
            return
        
        try:
            # Convert message based on schema type
            if schema_type == "auto":
                # Auto-detect schema type (simple heuristics)
                converted_msg = self._auto_detect_and_convert(channel, data)
            else:
                # Use specified converter
                converter = self.type_converters.get(schema_type)
                if converter:
                    converted_msg = converter(channel, data)
                else:
                    # Fallback to JSON
                    logger.warning(f"No converter for schema type: {schema_type}. Using JSON fallback.")
                    converted_msg = self._convert_json(channel, data)
            
            # print(f"Converted message: {converted_msg}")
            # Publish to Foxglove if conversion was successful
            if converted_msg is not None:
                foxglove_channel = self.channels[channel]
                if foxglove_channel:
                    foxglove_channel.log(converted_msg)
                    logger.debug(f"Published message to Foxglove for {channel}")
                else:
                    logger.warning(f"No Foxglove channel found for {channel}")
        except Exception as e:
            logger.error(f"Error converting/publishing message from {channel}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
    
    def _auto_detect_and_convert(self, channel: str, data: bytes) -> Any:
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
        
        if "transform" in channel_lower or "tf" in channel_lower:
            return self._convert_transform(channel, data)
        elif "point" in channel_lower and "cloud" in channel_lower:
            return self._convert_pointcloud(channel, data)
        elif "image" in channel_lower or "camera" in channel_lower:
            if "compressed" in channel_lower:
                return self._convert_compressed_image(channel, data)
            else:
                return self._convert_image(channel, data)
        elif "laser" in channel_lower or "scan" in channel_lower or "lidar" in channel_lower:
            return self._convert_laserscan(channel, data)
        elif "pose" in channel_lower:
            return self._convert_pose(channel, data)
        elif "location" in channel_lower or "gps" in channel_lower or "fix" in channel_lower:
            return self._convert_location(channel, data)
        elif "log" in channel_lower:
            return self._convert_log(channel, data)
        elif "vector3" in channel_lower:
            return self._convert_vector3(channel, data)  # Use our Vector3 JSON converter
        else:
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
        # Implement conversion from LCM transform to Foxglove FrameTransform
        # This is a placeholder - actual implementation depends on the LCM message structure
        
        # Parse LCM message - needs actual implementation based on your LCM message definition
        # For example, using lcm.decode() if you have the message class
        try:
            # Assuming 'msg' is the parsed LCM message with transform data
            # msg = lcm.decode(data, YourTransformMessageType)
            
            # For now, creating a dummy transform
            logger.warning(f"Using dummy implementation for transform conversion on {channel}")
            
            timestamp = Timestamp.from_seconds(time.time())
            
            # Create Foxglove FrameTransform
            transform = FrameTransform(
                timestamp=timestamp,
                parent_frame_id="world",
                child_frame_id="dummy_frame",
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            
            return transform
        except Exception as e:
            logger.error(f"Error converting transform message: {e}")
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
        # Similar to _convert_transform but for multiple transforms
        try:
            # Dummy implementation
            logger.warning(f"Using dummy implementation for transforms conversion on {channel}")
            
            timestamp = Timestamp.from_seconds(time.time())
            
            transform1 = FrameTransform(
                timestamp=timestamp,
                parent_frame_id="world",
                child_frame_id="frame1",
                translation=Vector3(x=1.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            
            transform2 = FrameTransform(
                timestamp=timestamp,
                parent_frame_id="world",
                child_frame_id="frame2",
                translation=Vector3(x=0.0, y=1.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            
            # Create Foxglove FrameTransforms
            transforms = FrameTransforms(
                transforms=[transform1, transform2]
            )
            
            return transforms
        except Exception as e:
            logger.error(f"Error converting transforms message: {e}")
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
            
            # Log the structure of the foxglove RawImage object for debugging
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f"Foxglove RawImage structure: {dir(raw_image)}")
            
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
            # Set up channels and subscriptions
            self._setup_channels()
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
        
        # Wait for threads to finish
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
    parser = argparse.ArgumentParser(description='LCM to Foxglove Bridge')
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
    logger.info("Bridge running. Press Ctrl+C to exit.")
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
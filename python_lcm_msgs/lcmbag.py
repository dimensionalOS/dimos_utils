#!/usr/bin/env python3
"""
LCM Bag - A tool for recording and playback of LCM messages

This tool provides functionality similar to rosbag2 but for LCM messages,
using MCAP as the storage format.

Commands:
    record  - Record LCM messages to a bag file
    play    - Play back messages from a bag file
    info    - Display information about a bag file
    list    - List available topics in a bag file
    
Interactive playback controls:
    - Pause/resume with spacebar
    - Play next message with 'n' when paused
    - Burst playback by 1 second with 'b' when paused
    - Burst playback by 5 seconds with 'B' when paused
    - Adjust playback speed with '.' (faster) and ',' (slower)
    - Reset playback speed to 1x with 'r'
    - Seek to percentage with number keys (0=start, 9=90%, 5=50%)
    - Seek backwards 5 seconds with '['
    - Seek forwards 5 seconds with ']'
    
Advanced usage:
    - Start paused: lcmbag.py play --pause bag.mcap
    - Burst playback: lcmbag.py play --pause --burst 3.0 bag.mcap
    - Seek to timestamp: (API available, use through custom scripts)
"""
# TODO:
# - Fix seek to percentage
# - Unknown message types still record and playback properly, but there should be better handling for this

import os
import sys
import time
import argparse
import threading
import json
import signal
import socket
import datetime
from typing import Dict, List, Any, Optional, Set, Tuple
from dataclasses import dataclass
import logging

try:
    import lcm
    import mcap
    from mcap.writer import Writer as MCAPWriter
    from mcap.reader import make_reader as make_mcap_reader
    import mcap.records as mcap_records
except ImportError:
    print("Error: Required packages not found. Please install using:")
    print("  pip install lcm-python mcap")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('lcmbag')

# Constants
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667"
DEFAULT_BAG_NAME = f"lcmbag_{datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')}"

@dataclass
class LCMMessageMeta:
    """Metadata for an LCM message."""
    channel: str
    timestamp: int
    data_type: str
    data: bytes

class LCMBagRecorder:
    """Records LCM messages to an MCAP file with optimizations for high-bandwidth streams."""
    
    # Pause modes
    PAUSE_NONE = 0     # Not paused
    PAUSE_SOFT = 1     # Standard pause - stop recording but keep time running
    PAUSE_FREEZE = 2   # Time-freeze pause - stop recording and pause elapsed time
    
    # Recording modes
    MODE_NORMAL = 0    # Normal recording mode - write messages to disk as they come in
    MODE_SNAPSHOT = 1  # Snapshot mode - keep messages in memory until snapshot is triggered
    
    def __init__(self, output_path: str, lcm_url: str = DEFAULT_LCM_URL, 
                 topics: List[str] = None, max_file_size_mb: int = 0,
                 buffer_capacity: int = 10000, stats_interval: int = 1000,
                 snapshot_mode: bool = False, max_cache_size: int = 1000000):
        """
        Initialize the recorder.
        
        Args:
            output_path: Path where the bag file will be written
            lcm_url: LCM URL to subscribe to
            topics: List of topics to record (None for all topics)
            max_file_size_mb: Maximum size of each file chunk in MB (0 for unlimited)
            buffer_capacity: Maximum number of messages to buffer before writing
            stats_interval: Number of messages between statistics logging
            snapshot_mode: If True, operate in snapshot mode (don't write until triggered)
            max_cache_size: Maximum number of messages to keep in snapshot circular buffer
        """
        self.output_path = output_path
        self.lcm_url = lcm_url
        self.topics = topics or []
        self.record_all = len(self.topics) == 0
        self.max_file_size_mb = max_file_size_mb
        self.buffer_capacity = buffer_capacity
        self.stats_interval = stats_interval
        self.recording_mode = self.MODE_SNAPSHOT if snapshot_mode else self.MODE_NORMAL
        self.max_cache_size = max_cache_size
        
        # State
        self.is_recording = False
        self.pause_mode = self.PAUSE_NONE
        self.lc = None
        self.writer = None
        self.subscription = None
        self.record_thread = None
        self.write_thread = None
        self.channel_schema_ids: Dict[str, int] = {}
        self.message_count = 0
        self.dropped_messages = 0
        self.start_time = 0
        self.stop_requested = False
        self.current_chunk = 0
        
        # Time tracking for pause modes
        self.elapsed_time = 0       # Total time including pauses
        self.active_time = 0        # Time only when actively recording
        self.pause_start_time = 0   # When the current pause started
        self.total_pause_duration = 0  # Total time spent in PAUSE_FREEZE mode
        self.last_time_update = 0   # Last time the elapsed time was updated
        
        # Message buffer for async writing (or snapshot circular buffer)
        self.message_buffer = []
        self.buffer_lock = threading.Lock()
        self.buffer_semaphore = threading.Semaphore(0)  # Signal when messages are available
        
        # Snapshot mode state
        self.snapshot_count = 0     # Number of snapshots taken
        self.snapshot_requested = threading.Event()  # Signal when snapshot is requested
        self._snapshot_in_progress = False  # Flag to prevent concurrent snapshots
        self._last_snapshot_time = 0  # Time of last snapshot request (for debouncing)
        
        # Performance metrics
        self.last_stats_time = 0
        self.processed_since_last_stats = 0
        self.message_rates = {}  # Per-topic message rates
        
        # Extended type mapping
        self.type_mapping = {
            "TWIST": "geometry_msgs/Twist",
            "VECTOR": "geometry_msgs/Vector3",
            "POSE": "geometry_msgs/Pose",
            "POINT": "geometry_msgs/Point",
            "QUATERNION": "geometry_msgs/Quaternion",
            "IMU": "sensor_msgs/Imu",
            "LASER": "sensor_msgs/LaserScan",
            "ODOMETRY": "nav_msgs/Odometry",
            "IMAGE": "sensor_msgs/Image",
            "COMPRESSED": "sensor_msgs/CompressedImage",
            "CAMERA": "sensor_msgs/CameraInfo",
            "MAP": "nav_msgs/OccupancyGrid",
            "PATH": "nav_msgs/Path",
            "TRANSFORM": "geometry_msgs/Transform",
            "MARKER": "visualization_msgs/Marker",
            "POINTCLOUD": "sensor_msgs/PointCloud2",
            "JOY": "sensor_msgs/Joy",
            "SCAN": "sensor_msgs/LaserScan",
            "TF": "tf2_msgs/TFMessage"
        }

        # Ensure output directory exists
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    def _get_chunk_filename(self, chunk_num=None):
        """Get filename for a specific chunk."""
        if chunk_num is None:
            chunk_num = self.current_chunk
            
        if self.max_file_size_mb <= 0:
            return self.output_path
            
        # Add chunk number to filename
        base, ext = os.path.splitext(self.output_path)
        return f"{base}_chunk{chunk_num}{ext}"

    def _create_lcm_subscription(self):
        """Create an LCM subscription for the requested topics."""
        self.lc = lcm.LCM(self.lcm_url)
        
        # Subscribe to either all topics or specific ones
        if self.record_all:
            self.subscription = self.lc.subscribe(".*", self._on_message)
        else:
            for topic in self.topics:
                self.subscription = self.lc.subscribe(topic, self._on_message)
                
        # Set larger receive buffer for UDP socket if possible
        # This helps prevent dropped packets for high-bandwidth streams
        try:
            self.lc.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 26214400)  # 25MB
            logger.info("Increased LCM receive buffer size")
        except (AttributeError, NameError):
            logger.debug("Could not increase LCM receive buffer size")
    
    def _create_mcap_writer(self, filename=None):
        """Create an MCAP writer for the output file."""
        if filename is None:
            filename = self._get_chunk_filename()
            
        try:
            self.writer = MCAPWriter(open(filename, 'wb'))
            self.writer.start(
                profile="",
                library="lcmbag.py",
            )
            logger.info(f"Created MCAP writer for {filename}")
        except Exception as e:
            logger.error(f"Failed to create MCAP writer: {e}")
            raise
    
    def _determine_message_type(self, channel: str) -> str:
        """Determine message type based on channel name."""
        # Normalize the channel name to simplify matching
        channel_lower = channel.lower()
        channel_upper = channel.upper()
        
        # Try direct matches first (case insensitive)
        for key, msg_type in self.type_mapping.items():
            if channel_upper == key or channel == key:
                return msg_type
            
        # Try prefix matches (case insensitive)
        for prefix, msg_type in self.type_mapping.items():
            if channel_upper.startswith(prefix):
                return msg_type
                
        # Try substring matches (more flexible but less precise)
        for key, msg_type in self.type_mapping.items():
            if key in channel_upper:
                return msg_type
                
        # Try word matching with underscores as separators (for cases like "thing1_vector3")
        channel_parts = channel_lower.split('_')
        for part in channel_parts:
            for key, msg_type in self.type_mapping.items():
                if part.upper() == key or key in part.upper():
                    return msg_type

        # Try word matching with dashes as separators (for cases like "thing1-vector3")
        channel_parts = channel_lower.split('-')
        for part in channel_parts:
            for key, msg_type in self.type_mapping.items():
                if part.upper() == key or key in part.upper():
                    return msg_type
            
        # Fall back to topic-specific heuristics for well-known message types
        
        # Geometry messages
        if "vec" in channel_lower or "vector" in channel_lower:
            return "geometry_msgs/Vector3"
        elif "quat" in channel_lower:
            return "geometry_msgs/Quaternion"
        elif "pose" in channel_lower:
            if "stamped" in channel_lower:
                return "geometry_msgs/PoseStamped"
            elif "array" in channel_lower:
                return "geometry_msgs/PoseArray"
            elif "covariance" in channel_lower:
                return "geometry_msgs/PoseWithCovariance"
            else:
                return "geometry_msgs/Pose"
        elif "point" in channel_lower:
            if "stamped" in channel_lower:
                return "geometry_msgs/PointStamped"
            elif "cloud" in channel_lower:
                if "2" in channel_lower:
                    return "sensor_msgs/PointCloud2"
                else:
                    return "sensor_msgs/PointCloud"
            else:
                return "geometry_msgs/Point"
        elif "transform" in channel_lower:
            if "stamped" in channel_lower:
                return "geometry_msgs/TransformStamped"
            else:
                return "geometry_msgs/Transform"
        elif "twist" in channel_lower:
            if "stamped" in channel_lower:
                return "geometry_msgs/TwistStamped"
            elif "covariance" in channel_lower:
                return "geometry_msgs/TwistWithCovariance"
            else:
                return "geometry_msgs/Twist"
        
        # Sensor messages
        elif "imu" in channel_lower:
            return "sensor_msgs/Imu"
        elif "scan" in channel_lower or "laser" in channel_lower or "lidar" in channel_lower:
            return "sensor_msgs/LaserScan"
        elif "img" in channel_lower or "image" in channel_lower:
            # Check if it's a compressed image
            if "compressed" in channel_lower:
                return "sensor_msgs/CompressedImage"
            else:
                return "sensor_msgs/Image"
        elif "cam" in channel_lower or "camera" in channel_lower:
            # Check if it's camera info
            if "info" in channel_lower:
                return "sensor_msgs/CameraInfo"
            else:
                return "sensor_msgs/Image"
        
        # Navigation messages
        elif "odom" in channel_lower:
            return "nav_msgs/Odometry"
        elif "map" in channel_lower or "grid" in channel_lower or "occupancy" in channel_lower:
            return "nav_msgs/OccupancyGrid"
        elif "path" in channel_lower:
            return "nav_msgs/Path"
            
        # Standard messages (if we can detect them)
        elif "bool" in channel_lower:
            return "std_msgs/Bool"
        elif "string" in channel_lower or "text" in channel_lower:
            return "std_msgs/String"
        elif "float" in channel_lower or "double" in channel_lower:
            if "array" in channel_lower:
                return "std_msgs/Float64MultiArray"
            else:
                return "std_msgs/Float64"
        elif "int" in channel_lower:
            if "array" in channel_lower:
                return "std_msgs/Int32MultiArray"
            else:
                return "std_msgs/Int32"
                
        # Default
        logger.debug(f"Could not determine type for channel: {channel}")
        return "unknown"
    
    def _on_message(self, channel, data):
        """Callback when an LCM message is received."""
        if self.stop_requested or self.pause_mode != self.PAUSE_NONE:
            return
        
        current_time = time.time()
        
        # Calculate the adjusted time based on freeze pauses
        # This ensures timestamps account for time-freeze periods
        adjusted_time = current_time - self.total_pause_duration
        
        # Get timestamp in nanoseconds, adjusted for time-freeze pauses
        timestamp_ns = int(adjusted_time * 1e9)
        
        # Initialize channel ID to None
        channel_id = None
        
        # In normal mode, or if a writer exists, register channel schema
        if self.recording_mode == self.MODE_NORMAL or self.writer:
            if channel in self.channel_schema_ids:
                channel_id = self.channel_schema_ids[channel]
            else:
                # Only register schema when we have a writer
                if self.writer:
                    message_type = self._determine_message_type(channel)
                    schema_name = message_type
                    
                    try:
                        # Register schema
                        schema_id = self.writer.register_schema(
                            name=schema_name,
                            encoding="lcm",
                            data=f"type:{message_type}".encode()
                        )
                        
                        # Register channel
                        channel_id = self.writer.register_channel(
                            topic=channel,
                            schema_id=schema_id,
                            message_encoding="lcm",
                        )
                        
                        self.channel_schema_ids[channel] = channel_id
                        if message_type == "unknown":
                            logger.warning(f"Registered channel {channel} with unknown type - type detection failed")
                            # Log the channel segments to help understand why detection failed
                            if '_' in channel or '-' in channel:
                                segments = []
                                if '_' in channel:
                                    segments.extend(channel.split('_'))
                                if '-' in channel:
                                    segments.extend(channel.split('-'))
                                logger.debug(f"Channel segments: {segments}")
                        else:
                            logger.info(f"Registered channel {channel} with type {message_type}")
                        
                        # Initialize message rate tracking for this channel
                        self.message_rates[channel] = {"count": 0, "last_time": current_time}
                        
                    except Exception as e:
                        logger.error(f"Failed to register channel {channel}: {e}")
                        return
        
        # Create message object with all necessary information
        message_obj = {
            "channel": channel,
            "channel_id": channel_id,
            "timestamp": timestamp_ns,  # Using adjusted timestamp!
            "data": data,
            "real_time": int(current_time * 1e9),  # Store real time for debugging
            "message_type": self._determine_message_type(channel)  # Store message type for later channel registration
        }
        
        # Add message to buffer, with different behavior based on recording mode
        with self.buffer_lock:
            if self.recording_mode == self.MODE_NORMAL:
                # Normal mode: check buffer capacity and send to writer
                if len(self.message_buffer) >= self.buffer_capacity:
                    self.dropped_messages += 1
                    if self.dropped_messages % 1000 == 1:
                        logger.warning(f"Buffer full, dropping messages ({self.dropped_messages} dropped so far)")
                    return
                    
                # Add to buffer and signal writer
                self.message_buffer.append(message_obj)
                self.buffer_semaphore.release()
            else:
                # Snapshot mode: maintain circular buffer of max_cache_size
                self.message_buffer.append(message_obj)
                
                # If buffer exceeds max size, remove oldest messages
                while len(self.message_buffer) > self.max_cache_size:
                    self.message_buffer.pop(0)
                    
                # No need to signal writer in snapshot mode, as we only write on snapshot request
            
            # Track message rate for statistics
            if channel in self.message_rates:
                self.message_rates[channel]["count"] += 1
            
        # Track overall statistics
        self.message_count += 1
        self.processed_since_last_stats += 1
        
        # Update active recording time
        if self.last_time_update > 0:
            time_delta = current_time - self.last_time_update
            self.active_time += time_delta
            # For non-frozen pause mode, elapsed time keeps counting even during pauses
            self.elapsed_time += time_delta
        self.last_time_update = current_time
        
        # Log statistics periodically
        if self.message_count % self.stats_interval == 0 or current_time - self.last_stats_time >= 10.0:
            time_diff = current_time - self.last_stats_time
            if time_diff > 0:
                msgs_per_sec = self.processed_since_last_stats / time_diff
                logger.info(f"Recording at {msgs_per_sec:.1f} msgs/sec, total: {self.message_count} msgs")
                
                # Log per-topic rates for high-frequency topics
                for channel, stats in self.message_rates.items():
                    channel_time_diff = current_time - stats["last_time"]
                    if channel_time_diff > 0 and stats["count"] > 0:
                        channel_rate = stats["count"] / channel_time_diff
                        if channel_rate > 10:  # Only log higher frequency topics
                            logger.info(f"Channel {channel}: {channel_rate:.1f} msgs/sec")
                        stats["count"] = 0
                        stats["last_time"] = current_time
                
                self.last_stats_time = current_time
                self.processed_since_last_stats = 0
    
    def _writer_loop(self):
        """Background thread for writing messages from buffer to MCAP file."""
        logger.info("Writer thread started")
        last_check_time = time.time()
        messages_written = 0
        
        # Debug info for time adjustments
        last_adjusted_time = 0
        time_gaps = []
        
        try:
            while not self.stop_requested:
                # In snapshot mode, wait for snapshot request instead of buffer semaphore
                if self.recording_mode == self.MODE_SNAPSHOT:
                    # Wait for snapshot request with timeout for periodic checks
                    snapshot_triggered = self.snapshot_requested.wait(timeout=1.0)
                    
                    if self.stop_requested:
                        break
                    
                    if snapshot_triggered:
                        # Note: Don't clear the event here - it will be cleared in _take_snapshot
                        # after processing completes
                        
                        # Take a snapshot - create a new file and write all buffered messages
                        self._take_snapshot()
                else:
                    # Normal mode - wait for messages to be available
                    acquired = self.buffer_semaphore.acquire(timeout=1.0)
                    
                    if self.stop_requested:
                        break
                    
                    if acquired:
                        # Get messages from buffer
                        messages_to_write = []
                        with self.buffer_lock:
                            # Take up to 1000 messages in one batch for efficiency
                            batch_size = min(1000, len(self.message_buffer))
                            if batch_size > 0:
                                messages_to_write = self.message_buffer[:batch_size]
                                self.message_buffer = self.message_buffer[batch_size:]
                    
                        # Write messages to file
                        if messages_to_write and self.writer:
                            self._write_messages_to_file(messages_to_write)
                            messages_written += len(messages_to_write)
                
                # Check if we need to rotate file (once per second)
                current_time = time.time()
                if self.max_file_size_mb > 0 and current_time - last_check_time >= 1.0:
                    last_check_time = current_time
                    
                    try:
                        # Get current file size
                        current_file = self._get_chunk_filename()
                        if os.path.exists(current_file):
                            file_size_mb = os.path.getsize(current_file) / (1024 * 1024)
                            
                            # Rotate file if needed
                            if file_size_mb >= self.max_file_size_mb:
                                logger.info(f"Rotating file: {file_size_mb:.1f}MB exceeds {self.max_file_size_mb}MB limit")
                                self._rotate_file()
                    except Exception as e:
                        logger.error(f"Error checking file size: {e}")
                        
        except Exception as e:
            logger.error(f"Writer thread error: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            if time_gaps:
                avg_gap = sum(time_gaps) / len(time_gaps)
                logger.debug(f"Detected {len(time_gaps)} time gaps, average: {avg_gap:.2f}s")
            logger.info(f"Writer thread exiting. Wrote {messages_written} messages.")
    
    def _write_messages_to_file(self, messages_to_write):
        """Write a batch of messages to the MCAP file."""
        last_adjusted_time = 0
        time_gaps = []
        
        for msg in messages_to_write:
            try:
                # First ensure channel is registered
                channel = msg["channel"]
                channel_id = msg["channel_id"]
                
                # If channel_id is None but we have a writer, register the channel now
                if channel_id is None and self.writer and channel not in self.channel_schema_ids:
                    message_type = msg.get("message_type", self._determine_message_type(channel))
                    schema_name = message_type
                    
                    # Register schema
                    schema_id = self.writer.register_schema(
                        name=schema_name,
                        encoding="lcm",
                        data=f"type:{message_type}".encode()
                    )
                    
                    # Register channel
                    channel_id = self.writer.register_channel(
                        topic=channel,
                        schema_id=schema_id,
                        message_encoding="lcm",
                    )
                    
                    self.channel_schema_ids[channel] = channel_id
                    if message_type == "unknown":
                        logger.warning(f"Registered channel {channel} with unknown type - type detection failed")
                        # Log the channel segments to help understand why detection failed
                        if '_' in channel or '-' in channel:
                            segments = []
                            if '_' in channel:
                                segments.extend(channel.split('_'))
                            if '-' in channel:
                                segments.extend(channel.split('-'))
                            logger.debug(f"Channel segments: {segments}")
                    else:
                        logger.info(f"Registered channel {channel} with type {message_type}")
                
                # Now write the message if we have a channel ID
                if channel_id is not None:
                    # Use adjusted timestamps for both log and publish time
                    adjusted_time = msg["timestamp"]
                    
                    # Debug: track time gaps to detect pause boundaries
                    if last_adjusted_time > 0 and adjusted_time - last_adjusted_time > 1e9:  # gap > 1 second
                        time_gap_s = (adjusted_time - last_adjusted_time) / 1e9
                        time_gaps.append(time_gap_s)
                        if len(time_gaps) <= 3:  # Only log a few gaps to avoid spam
                            logger.debug(f"Time gap detected: {time_gap_s:.2f}s between messages")
                    
                    last_adjusted_time = adjusted_time
                    
                    self.writer.add_message(
                        channel_id=channel_id,
                        log_time=adjusted_time,
                        publish_time=adjusted_time,
                        data=msg["data"]
                    )
            except Exception as e:
                logger.error(f"Error writing message: {e}")
                
    def _take_snapshot(self):
        """Take a snapshot - create a new file and write all buffered messages."""
        try:
            # Create a mutex to ensure only one snapshot process at a time
            if hasattr(self, '_snapshot_in_progress') and self._snapshot_in_progress:
                logger.warning("Another snapshot is already in progress, ignoring duplicate request")
                return
                
            self._snapshot_in_progress = True
            
            with self.buffer_lock:
                if not self.message_buffer:
                    logger.warning("Snapshot requested but buffer is empty")
                    self._snapshot_in_progress = False
                    return
                    
                # Make a copy of the buffer to write - must be done inside lock
                messages_to_write = list(self.message_buffer)  # Create a new list with the same elements
            
            # Generate snapshot number and filename outside the lock
            self.snapshot_count += 1
            current_snapshot_num = self.snapshot_count
            
            # Create snapshot-specific filename
            snapshot_time = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
            base, ext = os.path.splitext(self.output_path)
            snapshot_file = f"{base}_snapshot{current_snapshot_num}_{snapshot_time}{ext}"
            
            logger.info(f"Taking snapshot #{current_snapshot_num} with {len(messages_to_write)} messages")
            logger.info(f"Writing to: {snapshot_file}")
            
            # Sort messages by timestamp to ensure proper playback order
            messages_to_write.sort(key=lambda m: m["timestamp"])
            
            # Create a local schema registry for this snapshot
            local_channel_ids = {}
            
            # Create a new writer for this snapshot
            snapshot_writer = None
            try:
                # Create a new writer
                snapshot_writer = MCAPWriter(open(snapshot_file, 'wb'))
                snapshot_writer.start(
                    profile="",
                    library="lcmbag.py - snapshot",
                )
                
                # Process messages in chronological order
                for msg in messages_to_write:
                    channel = msg["channel"]
                    
                    # Register channel and schema if needed
                    if channel not in local_channel_ids:
                        message_type = msg.get("message_type", self._determine_message_type(channel))
                        schema_name = message_type
                        
                        # Register schema
                        schema_id = snapshot_writer.register_schema(
                            name=schema_name,
                            encoding="lcm",
                            data=f"type:{message_type}".encode()
                        )
                        
                        # Register channel
                        channel_id = snapshot_writer.register_channel(
                            topic=channel,
                            schema_id=schema_id,
                            message_encoding="lcm",
                        )
                        
                        local_channel_ids[channel] = channel_id
                        logger.info(f"Registered channel {channel} with type {message_type}")
                    else:
                        channel_id = local_channel_ids[channel]
                    
                    # Write message to file
                    snapshot_writer.add_message(
                        channel_id=channel_id,
                        log_time=msg["timestamp"],
                        publish_time=msg["timestamp"],
                        data=msg["data"]
                    )
                
                # Close the writer
                snapshot_writer.finish()
                snapshot_writer = None
                
                logger.info(f"Snapshot #{current_snapshot_num} completed: {len(messages_to_write)} messages written to {snapshot_file}")
                
            except Exception as e:
                logger.error(f"Error taking snapshot: {e}")
                import traceback
                logger.error(traceback.format_exc())
                if snapshot_writer:
                    try:
                        snapshot_writer.finish()
                    except:
                        pass
        finally:
            # Reset snapshot request flag and in-progress flag
            self._snapshot_in_progress = False
            # Clear the request flag after the snapshot is complete
            self.snapshot_requested.clear()
    
    def _rotate_file(self):
        """Rotate to a new file chunk."""
        if not self.writer:
            return
            
        logger.info(f"Rotating file, closing chunk {self.current_chunk}")
        
        # Finish current writer
        try:
            self.writer.finish()
        except Exception as e:
            logger.error(f"Error finishing writer: {e}")
        
        # Create new writer
        self.current_chunk += 1
        self.writer = None
        self.channel_schema_ids = {}  # Reset schema IDs for new file
        
        try:
            new_file = self._get_chunk_filename()
            self._create_mcap_writer(new_file)
            logger.info(f"Created new chunk: {new_file}")
        except Exception as e:
            logger.error(f"Failed to create new writer: {e}")
    
    def _record_loop(self):
        """Main recording loop that handles LCM messages."""
        logger.info(f"Recording to: {self._get_chunk_filename()}")
        logger.info(f"Press Ctrl+C to stop recording")
        
        self.start_time = time.time()
        self.last_stats_time = self.start_time
        self.last_time_update = self.start_time
        
        try:
            # Start background writer thread
            self.write_thread = threading.Thread(target=self._writer_loop)
            self.write_thread.daemon = True
            self.write_thread.start()
            
            # LCM message handler thread
            while not self.stop_requested:
                try:
                    # Update time tracking based on pause mode
                    current_time = time.time()
                    
                    # For standard pause (PAUSE_SOFT), we still update elapsed time
                    if self.pause_mode == self.PAUSE_SOFT:
                        if self.last_time_update > 0:
                            time_delta = current_time - self.last_time_update
                            self.elapsed_time += time_delta
                        self.last_time_update = current_time
                    
                    # For time-freeze pause (PAUSE_FREEZE), track how long we've been paused
                    elif self.pause_mode == self.PAUSE_FREEZE:
                        if self.pause_start_time == 0:
                            # Just entered FREEZE mode, record the time
                            self.pause_start_time = current_time
                        # Don't update last_time_update while in freeze mode
                    
                    # For active recording, time tracking is handled in _on_message
                    
                    # Process LCM messages with timeout to check for stop flag
                    timeout_ms = 100
                    self.lc.handle_timeout(timeout_ms)
                except Exception as e:
                    logger.error(f"Error handling LCM messages: {e}")
                    # Brief pause to avoid spinning too fast if there are errors
                    time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            logger.info("Record loop exiting")
            self.stop()
    
    def start(self):
        """Start recording."""
        if self.is_recording:
            logger.warning("Already recording")
            return
        
        try:
            self._create_lcm_subscription()
            
            # In normal mode, create the writer right away
            # In snapshot mode, writer is created only when taking a snapshot
            if self.recording_mode == self.MODE_NORMAL:
                self._create_mcap_writer()
                
            self.is_recording = True
            self.stop_requested = False
            self.pause_mode = self.PAUSE_NONE
            self.message_count = 0
            self.dropped_messages = 0
            self.message_buffer = []
            self.current_chunk = 0
            self.snapshot_count = 0
            self.snapshot_requested.clear()
            
            # Reset time tracking
            self.elapsed_time = 0
            self.active_time = 0
            self.pause_start_time = 0
            self.total_pause_duration = 0

            # Start record and writer threads
            self.record_thread = threading.Thread(target=self._record_loop)
            self.record_thread.daemon = True
            self.record_thread.start()
            
            self.write_thread = threading.Thread(target=self._writer_loop)
            self.write_thread.daemon = True
            self.write_thread.start()
            
            if self.recording_mode == self.MODE_SNAPSHOT:
                logger.info(f"Snapshot recording started - buffering up to {self.max_cache_size} messages")
            else:
                logger.info("Recording started")
        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            self.stop()
            raise
            
    def trigger_snapshot(self):
        """
        Trigger a snapshot - save the current circular buffer to disk.
        Only applicable in snapshot mode.
        
        Returns:
            bool: True if snapshot was triggered, False otherwise
        """
        if not self.is_recording:
            logger.warning("Not recording - cannot take snapshot")
            return False
            
        if self.recording_mode != self.MODE_SNAPSHOT:
            logger.warning("Not in snapshot mode - snapshots can only be triggered in snapshot mode")
            return False
            
        if self.pause_mode != self.PAUSE_NONE:
            logger.warning("Recording is paused - cannot take snapshot")
            return False
            
        # Check if a snapshot is already being processed
        if self.snapshot_requested.is_set():
            logger.warning("A snapshot is already being processed - ignoring duplicate request")
            return False
            
        # Signal the writer thread to take a snapshot
        logger.info("Snapshot requested")
        self.snapshot_requested.set()
        return True
    
    def stop(self):
        """Stop recording."""
        if not self.is_recording:
            return
            
        logger.info("Stopping recording...")
        self.stop_requested = True
        
        # First ensure we're not paused - compute final time tracking if needed
        if self.pause_mode == self.PAUSE_FREEZE:
            # If we were in freeze pause mode, calculate the total pause duration
            if self.pause_start_time > 0:
                pause_duration = time.time() - self.pause_start_time
                self.total_pause_duration += pause_duration
        
        # Update final elapsed time
        if self.pause_mode != self.PAUSE_FREEZE:
            self.elapsed_time = time.time() - self.start_time - self.total_pause_duration
        
        # Wait for record thread to finish
        if self.record_thread and self.record_thread.is_alive() and self.record_thread != threading.current_thread():
            self.record_thread.join(timeout=3.0)
        
        # Wait for writer thread to finish
        if self.write_thread and self.write_thread.is_alive():
            self.write_thread.join(timeout=3.0)
        
        # Write any remaining messages in the buffer
        if self.writer and self.message_buffer:
            logger.info(f"Writing remaining {len(self.message_buffer)} messages")
            try:
                for msg in self.message_buffer:
                    if msg["channel_id"] is not None:
                        self.writer.add_message(
                            channel_id=msg["channel_id"],
                            log_time=msg["timestamp"],
                            publish_time=msg["timestamp"],
                            data=msg["data"]
                        )
            except Exception as e:
                logger.error(f"Error writing remaining messages: {e}")
        
        self.is_recording = False
        
        # Compute recording duration with accurate time tracking
        real_duration = time.time() - self.start_time
        active_duration = self.elapsed_time
        
        # Finalize the MCAP file
        if self.writer:
            try:
                self.writer.finish()
            except Exception as e:
                logger.error(f"Error finishing writer: {e}")
            finally:
                self.writer = None
        
        # Close LCM
        if self.lc:
            self.lc = None
        
        # Report both total and active durations
        logger.info(f"Recording stopped. Recorded {self.message_count} messages")
        logger.info(f"Total duration: {real_duration:.2f} seconds")
        logger.info(f"Active recording time: {active_duration:.2f} seconds")
        if self.total_pause_duration > 0:
            logger.info(f"Time paused: {self.total_pause_duration:.2f} seconds")
        if self.dropped_messages > 0:
            logger.warning(f"Dropped {self.dropped_messages} messages due to buffer overflow")
    
    def pause(self, freeze_time=False):
        """
        Pause recording.
        
        Args:
            freeze_time: If True, also pauses the elapsed time counter (time-freeze mode)
                         If False, stops recording but keeps counting elapsed time (standard mode)
        """
        if not self.is_recording:
            logger.warning("Not recording")
            return
            
        # Already in some pause mode
        if self.pause_mode != self.PAUSE_NONE:
            current_mode = "standard pause" if self.pause_mode == self.PAUSE_SOFT else "time-freeze pause"
            requested_mode = "time-freeze pause" if freeze_time else "standard pause"
            logger.warning(f"Already in {current_mode} mode")
            
            # Allow switching between pause modes
            if (self.pause_mode == self.PAUSE_SOFT and freeze_time) or (self.pause_mode == self.PAUSE_FREEZE and not freeze_time):
                old_mode = self.pause_mode
                
                # If switching from FREEZE to SOFT, add the frozen time to total_pause_duration
                if old_mode == self.PAUSE_FREEZE:
                    freeze_duration = time.time() - self.pause_start_time
                    self.total_pause_duration += freeze_duration
                    self.pause_start_time = 0
                    logger.info(f"Added {freeze_duration:.2f}s to total pause time")
                
                # If switching from SOFT to FREEZE, record the start time
                if freeze_time:
                    self.pause_mode = self.PAUSE_FREEZE
                    self.pause_start_time = time.time()
                else:
                    self.pause_mode = self.PAUSE_SOFT
                
                logger.info(f"Switched from {old_mode} to {self.pause_mode} pause mode")
            return
            
        # Start the appropriate pause mode
        if freeze_time:
            self.pause_mode = self.PAUSE_FREEZE
            self.pause_start_time = time.time()
            logger.info("Recording paused with time freeze")
        else:
            self.pause_mode = self.PAUSE_SOFT
            logger.info("Recording paused (standard mode)")
            
        # Update time tracking when entering pause mode
        if self.last_time_update > 0:
            time_delta = time.time() - self.last_time_update
            self.active_time += time_delta
            self.elapsed_time += time_delta
            self.last_time_update = time.time()
    
    def resume(self):
        """Resume recording from any pause mode."""
        if not self.is_recording:
            logger.warning("Not recording")
            return
            
        if self.pause_mode == self.PAUSE_NONE:
            logger.warning("Not paused")
            return
        
        # If resuming from time-freeze pause, add the pause duration to total_pause_duration
        if self.pause_mode == self.PAUSE_FREEZE and self.pause_start_time > 0:
            freeze_duration = time.time() - self.pause_start_time
            self.total_pause_duration += freeze_duration
            logger.debug(f"Added {freeze_duration:.2f}s to total pause time (now {self.total_pause_duration:.2f}s)")
            
        # Reset the last time update when resuming
        self.last_time_update = time.time()
        self.pause_start_time = 0
        
        # Resume recording
        old_mode = self.pause_mode
        self.pause_mode = self.PAUSE_NONE
        logger.info(f"Recording resumed from {old_mode}")
        
    def get_elapsed_time(self):
        """
        Get the current elapsed recording time, accounting for pause modes.
        
        Returns:
            float: Elapsed time in seconds
        """
        if not self.is_recording:
            return 0
            
        current_time = time.time()
        
        # If not recording or in standard pause, just use the current elapsed time
        if self.pause_mode == self.PAUSE_NONE or self.pause_mode == self.PAUSE_SOFT:
            return current_time - self.start_time - self.total_pause_duration
            
        # If in time-freeze pause, subtract the current pause duration
        if self.pause_mode == self.PAUSE_FREEZE and self.pause_start_time > 0:
            current_pause = current_time - self.pause_start_time
            return current_time - self.start_time - self.total_pause_duration - current_pause
            
        return current_time - self.start_time - self.total_pause_duration


class LCMBagPlayer:
    """Plays back LCM messages from an MCAP file with optimizations for high-bandwidth streams."""
    
    def __init__(self, bag_path: str, lcm_url: str = DEFAULT_LCM_URL, 
                 topics: List[str] = None, rate: float = 1.0,
                 start_time: Optional[int] = None, end_time: Optional[int] = None,
                 loop: bool = False, chunk_size: int = 1000):
        """
        Initialize the player.
        
        Args:
            bag_path: Path to the bag file to play
            lcm_url: LCM URL to publish to
            topics: List of topics to play (None for all topics)
            rate: Playback rate multiplier (1.0 = realtime)
            start_time: Start playback from this timestamp (ns, None for beginning)
            end_time: End playback at this timestamp (ns, None for end)
            loop: Whether to loop playback when reaching the end
            chunk_size: Number of messages to load at once for large files
        """
        self.bag_path = bag_path
        self.lcm_url = lcm_url
        self.topics = set(topics) if topics else None
        self.rate = rate
        self.start_time_filter = start_time
        self.end_time_filter = end_time
        self.loop = loop
        self.chunk_size = chunk_size
        
        # State
        self.is_playing = False
        self.is_paused = False
        self.lc = None
        self.play_thread = None
        self.publish_thread = None
        self.stop_requested = False
        self.message_queue = []
        self.message_count = 0
        self.total_message_count = 0
        self.publish_queue = []
        self.queue_lock = threading.Lock()
        self.queue_semaphore = threading.Semaphore(0)
        self.current_index = 0
        
        # Burst control
        self.burst_requested = threading.Event()
        self.burst_seconds = 0
        self.play_next_requested = threading.Event()
        
        # Performance metrics
        self.messages_played = 0
        self.start_play_time = 0
        self.last_report_time = 0
        self.played_since_last_report = 0
        
        # Check if file exists
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Bag file not found: {self.bag_path}")
            
        # Get file info but don't load all messages yet
        self._analyze_bag()
    
    def _analyze_bag(self):
        """Analyze bag file to get message counts and time range."""
        logger.info(f"Analyzing bag file: {self.bag_path}")
        
        # Get file size
        file_size = os.path.getsize(self.bag_path)
        logger.info(f"Bag file size: {self._format_size(file_size)}")
        
        topic_counts = {}
        first_timestamp = None
        last_timestamp = None
        message_count = 0
        
        # Open the MCAP file
        with open(self.bag_path, 'rb') as f:
            reader = make_mcap_reader(f)
            
            # Scan the file to get statistics
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                message_count += 1
                
                # Track topic counts
                if topic not in topic_counts:
                    topic_counts[topic] = 0
                topic_counts[topic] += 1
                
                # Track time range
                if first_timestamp is None or message.log_time < first_timestamp:
                    first_timestamp = message.log_time
                
                if last_timestamp is None or message.log_time > last_timestamp:
                    last_timestamp = message.log_time
                    
                # Just read a sample of messages to get statistics
                if message_count >= 1000 and first_timestamp and last_timestamp:
                    logger.info("Sampled first 1000 messages for statistics")
                    break
        
        self.total_message_count = message_count
        self.first_timestamp = first_timestamp
        self.last_timestamp = last_timestamp
        self.topic_counts = topic_counts
        
        # Calculate time range
        if first_timestamp and last_timestamp:
            duration_ns = last_timestamp - first_timestamp
            duration_s = duration_ns / 1e9
            logger.info(f"Time range: {duration_s:.2f} seconds")
            
        # Apply time filters if specified
        self.effective_start_time = self.start_time_filter if self.start_time_filter else first_timestamp
        self.effective_end_time = self.end_time_filter if self.end_time_filter else last_timestamp
        
        # Log topic information
        filtered_topics = []
        logger.info(f"Found {len(topic_counts)} topics:")
        for topic, count in topic_counts.items():
            if not self.topics or topic in self.topics:
                logger.info(f"  {topic}: {count} messages")
                filtered_topics.append(topic)
            
        if self.topics:
            logger.info(f"Will play {len(filtered_topics)}/{len(topic_counts)} topics")
    
    def _format_size(self, size_bytes):
        """Format byte size to human readable format."""
        if size_bytes < 1024:
            return f"{size_bytes} B"
        elif size_bytes < 1024 * 1024:
            return f"{size_bytes / 1024:.1f} KiB"
        elif size_bytes < 1024 * 1024 * 1024:
            return f"{size_bytes / (1024 * 1024):.1f} MiB"
        else:
            return f"{size_bytes / (1024 * 1024 * 1024):.1f} GiB"
    
    def _load_messages_chunk(self, start_index=0, max_count=None):
        """
        Load a chunk of messages from the MCAP file.
        
        Args:
            start_index: Index to start loading from
            max_count: Maximum number of messages to load (None for all)
        
        Returns:
            Tuple of (messages, next_index)
        """
        if max_count is None:
            max_count = self.chunk_size
            
        logger.info(f"Loading messages chunk from index {start_index}, max {max_count}")
        
        messages = []
        next_index = start_index
        current_index = 0
        
        # Open the MCAP file
        with open(self.bag_path, 'rb') as f:
            reader = make_mcap_reader(f)
            
            # Skip to start_index
            for schema, channel, message in reader.iter_messages():
                if current_index >= start_index:
                    topic = channel.topic
                    
                    # Apply topic filter
                    if self.topics and topic not in self.topics:
                        current_index += 1
                        next_index = current_index
                        continue
                    
                    # Apply time filter
                    if (self.effective_start_time and message.log_time < self.effective_start_time) or \
                       (self.effective_end_time and message.log_time > self.effective_end_time):
                        current_index += 1
                        next_index = current_index
                        continue
                    
                    # Add message to queue
                    messages.append({
                        'channel': topic,
                        'timestamp': message.log_time,
                        'data': message.data
                    })
                    
                    next_index = current_index + 1
                    
                    # Check if we've loaded enough messages
                    if len(messages) >= max_count:
                        break
                
                current_index += 1
        
        # Sort messages by timestamp
        messages.sort(key=lambda m: m['timestamp'])
        logger.info(f"Loaded {len(messages)} messages, next index: {next_index}")
        
        return messages, next_index
    
    def _publisher_thread(self):
        """Thread that publishes messages from the publish queue."""
        logger.info("Publisher thread started")
        messages_published = 0
        
        try:
            while not self.stop_requested:
                # Wait for messages in the queue
                acquired = self.queue_semaphore.acquire(timeout=0.1)
                
                if self.stop_requested:
                    break
                    
                if not acquired:
                    continue
                
                # Get message from queue
                msg = None
                with self.queue_lock:
                    if self.publish_queue:
                        msg = self.publish_queue.pop(0)
                    else:
                        # No message in queue despite semaphore (rare race condition)
                        continue
                
                if msg:
                    # Check if this message has a "force_publish" flag (for play_next and burst)
                    force_publish = msg.get('force_publish', False)
                    
                    # If paused and not a forced publish, put the message back
                    if self.is_paused and not force_publish:
                        with self.queue_lock:
                            self.publish_queue.insert(0, msg)
                        # Wait a bit before trying again
                        time.sleep(0.01)
                        continue
                    
                    # Publish message
                    try:
                        channel = msg['channel']
                        data = msg['data']
                        self.lc.publish(channel, data)
                        messages_published += 1
                        self.messages_played += 1
                        self.played_since_last_report += 1
                        
                        # Report progress periodically
                        current_time = time.time()
                        if messages_published % 100 == 0 or current_time - self.last_report_time >= 5.0:
                            if current_time > self.last_report_time:
                                elapsed = current_time - self.last_report_time
                                if elapsed > 0:
                                    rate = self.played_since_last_report / elapsed
                                    logger.info(f"Published {self.messages_played}/{self.message_count} messages ({rate:.1f} msgs/sec)")
                                    self.last_report_time = current_time
                                    self.played_since_last_report = 0
                    except Exception as e:
                        logger.error(f"Error publishing message: {e}")
        
        except Exception as e:
            logger.error(f"Publisher thread error: {e}")
        finally:
            logger.info(f"Publisher thread exiting. Published {messages_published} messages.")
    
    def _play_loop(self):
        """Main playback loop that manages timing and message loading."""
        logger.info(f"Starting playback at rate {self.rate}x")
        if self.loop:
            logger.info("Playback will loop when complete")
        
        self.start_play_time = time.time()
        self.last_report_time = self.start_play_time
        
        # Track pause/resume state
        self.pause_start_time = 0
        self.total_pause_duration = 0
        
        try:
            # Start publisher thread
            self.publish_thread = threading.Thread(target=self._publisher_thread)
            self.publish_thread.daemon = True
            self.publish_thread.start()
            
            # Load initial chunk of messages
            chunk, next_index = self._load_messages_chunk(0, self.chunk_size)
            if not chunk:
                logger.warning("No messages to play (after filtering)")
                return
                
            self.message_queue = chunk
            self.message_count = len(chunk)
            self.current_index = 0
            
            # Get reference timestamps
            ref_bag_time = self.message_queue[0]['timestamp'] / 1e9  # Convert to seconds
            ref_real_time = time.time()
            
            # Main playback loop
            while not self.stop_requested:
                # Handle pause/resume timing
                if self.is_paused:
                    # If this is the first iteration of being paused, record the pause start time
                    if self.pause_start_time == 0:
                        self.pause_start_time = time.time()
                    
                    # Check for play_next request while paused
                    if self.play_next_requested.is_set():
                        logger.info("Processing play_next request")
                        self.play_next_requested.clear()
                        
                        # Process a single message
                        if self.current_index < len(self.message_queue):
                            msg = self.message_queue[self.current_index].copy()  # Create a copy to modify
                            # Mark this message for immediate publish even when paused
                            msg['force_publish'] = True
                            
                            with self.queue_lock:
                                if len(self.publish_queue) < 1000:
                                    self.publish_queue.append(msg)
                                    self.queue_semaphore.release()
                                else:
                                    logger.warning("Publisher queue full during play_next, skipping message")
                            
                            # Move to next message
                            self.current_index += 1
                            logger.info(f"Played next message, now at index {self.current_index}")
                        else:
                            logger.warning("No more messages to play in current chunk")
                    
                    # Check for burst request while paused
                    elif self.burst_requested.is_set():
                        logger.info(f"Processing burst request for {self.burst_seconds} seconds")
                        self.burst_requested.clear()
                        
                        # Calculate burst end time based on message timestamps
                        if self.current_index < len(self.message_queue):
                            current_msg = self.message_queue[self.current_index]
                            current_timestamp = current_msg['timestamp']
                            burst_end_timestamp = current_timestamp + int(self.burst_seconds * 1e9)
                            
                            burst_count = 0
                            # Process messages until we reach the burst duration or run out of messages
                            while (self.current_index < len(self.message_queue) and 
                                   self.message_queue[self.current_index]['timestamp'] <= burst_end_timestamp):
                                
                                msg = self.message_queue[self.current_index].copy()  # Create a copy to modify
                                # Mark this message for immediate publish even when paused
                                msg['force_publish'] = True
                                
                                with self.queue_lock:
                                    if len(self.publish_queue) < 1000:
                                        self.publish_queue.append(msg)
                                        self.queue_semaphore.release()
                                        burst_count += 1
                                    else:
                                        logger.warning("Publisher queue full during burst, skipping messages")
                                        break
                                
                                # Move to next message
                                self.current_index += 1
                                
                                # Check if we need to load more messages for the burst
                                if self.current_index >= len(self.message_queue) and next_index > 0 and next_index < self.total_message_count:
                                    # Load next chunk
                                    logger.info(f"Loading next chunk during burst at index {next_index}")
                                    chunk, next_chunk_index = self._load_messages_chunk(next_index, self.chunk_size)
                                    if chunk:
                                        self.message_queue = chunk
                                        self.current_index = 0
                                        next_index = next_chunk_index
                                        # Update counts
                                        self.message_count = len(self.message_queue)
                                    else:
                                        # No more messages
                                        break
                            
                            logger.info(f"Burst complete: published {burst_count} messages over {self.burst_seconds} seconds")
                        else:
                            logger.warning("No messages available for burst")
                    
                    # Regular pause - just sleep briefly
                    else:
                        time.sleep(0.01)
                    continue
                
                elif self.pause_start_time > 0:
                    # Just resumed from pause
                    pause_duration = time.time() - self.pause_start_time
                    self.total_pause_duration += pause_duration
                    logger.debug(f"Resumed after {pause_duration:.2f}s pause, total pause time: {self.total_pause_duration:.2f}s")
                    # Update reference time to account for the pause
                    ref_real_time += pause_duration
                    self.pause_start_time = 0
                
                # Check if we need to load more messages
                if self.current_index >= len(self.message_queue) and next_index > 0:
                    if next_index >= self.total_message_count:
                        if self.loop:
                            logger.info("End of bag reached, looping back to start")
                            self.current_index = 0
                            next_index = 0
                            # Reset reference times for smooth looping
                            if self.message_queue:
                                ref_bag_time = self.message_queue[0]['timestamp'] / 1e9
                                ref_real_time = time.time()
                                self.total_pause_duration = 0  # Reset pause tracking on loop
                            chunk, next_index = self._load_messages_chunk(0, self.chunk_size)
                            if chunk:
                                self.message_queue = chunk
                                self.message_count = len(chunk)
                            continue
                        else:
                            logger.info("Playback complete")
                            break
                    
                    # Load next chunk
                    logger.info(f"Loading next chunk at index {next_index}")
                    chunk, next_chunk_index = self._load_messages_chunk(next_index, self.chunk_size)
                    if chunk:
                        self.message_queue = chunk
                        self.current_index = 0
                        next_index = next_chunk_index
                        # Update counts
                        self.message_count = len(self.message_queue)
                    else:
                        # No more messages
                        break
                
                # Check if we've reached the end of the current chunk
                if self.current_index >= len(self.message_queue):
                    # Wait briefly and check again
                    time.sleep(0.01)
                    continue
                
                # Get current message
                msg = self.message_queue[self.current_index]
                
                # Calculate timing - account for accumulated pause time
                msg_time = msg['timestamp'] / 1e9  # Convert to seconds
                target_elapsed = (msg_time - ref_bag_time) / self.rate
                actual_elapsed = time.time() - ref_real_time
                
                # Check if it's time to publish this message
                if actual_elapsed < target_elapsed:
                    # Not yet time, sleep a bit
                    sleep_time = min(target_elapsed - actual_elapsed, 0.01)  # Max 10ms sleep for responsiveness
                    time.sleep(max(0, sleep_time))
                    continue
                
                # Add message to publish queue
                with self.queue_lock:
                    # Limit queue size to avoid memory issues
                    if len(self.publish_queue) < 1000:
                        self.publish_queue.append(msg)
                        self.queue_semaphore.release()
                    else:
                        logger.warning("Publisher queue full, skipping message")
                
                # Move to next message
                self.current_index += 1
            
            logger.info(f"Play loop complete. Queued {self.messages_played} messages")
                
        except Exception as e:
            logger.error(f"Playback error: {e}")
            import traceback
            logger.debug(traceback.format_exc())
        finally:
            self.stop()
    
    def start(self):
        """Start playback."""
        if self.is_playing:
            logger.warning("Already playing")
            return
        
        try:
            # Initialize LCM
            self.lc = lcm.LCM(self.lcm_url)
            
            self.is_playing = True
            self.stop_requested = False
            self.messages_played = 0
            self.played_since_last_report = 0
            self.publish_queue = []
            
            # Start playback thread
            self.play_thread = threading.Thread(target=self._play_loop)
            self.play_thread.daemon = True
            self.play_thread.start()
            logger.info("Playback started")
        except Exception as e:
            logger.error(f"Failed to start playback: {e}")
            self.stop()
            raise
    
    def stop(self):
        """Stop playback."""
        if not self.is_playing:
            return
            
        logger.info("Stopping playback...")
        self.stop_requested = True
        
        # Wait for threads to finish
        if self.play_thread and self.play_thread.is_alive() and self.play_thread != threading.current_thread():
            self.play_thread.join(timeout=3.0)
        
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=3.0)
        
        self.is_playing = False
        
        # Close LCM
        if self.lc:
            self.lc = None
        
        # Calculate stats
        if self.start_play_time > 0:
            duration = time.time() - self.start_play_time
            rate = self.messages_played / duration if duration > 0 else 0
            logger.info(f"Playback stopped. Played {self.messages_played} messages in {duration:.2f} seconds ({rate:.1f} msgs/sec)")
    
    def pause(self):
        """Pause playback."""
        if not self.is_playing:
            logger.warning("Not playing")
            return
            
        if self.is_paused:
            logger.warning("Already paused")
            return
        
        # Clear the publish queue to ensure we don't publish stale messages on resume
        with self.queue_lock:
            # Count how many messages were in the queue
            queue_size = len(self.publish_queue)
            if queue_size > 0:
                logger.debug(f"Clearing {queue_size} messages from queue on pause")
                self.publish_queue = []
                
                # Reset the semaphore by draining it
                for _ in range(queue_size):
                    try:
                        self.queue_semaphore.acquire(timeout=0.0001)
                    except:
                        pass
        
        self.is_paused = True
        logger.info("Playback paused")
    
    def resume(self):
        """Resume playback."""
        if not self.is_playing:
            logger.warning("Not playing")
            return
            
        if not self.is_paused:
            logger.warning("Not paused")
            return
            
        self.is_paused = False
        logger.info("Playback resumed")
    
    def burst(self, seconds: float):
        """
        Burst playback by specified number of seconds.
        Only works when player is paused.
        
        Args:
            seconds: Number of seconds to burst forward
        
        Returns:
            bool: True if burst was triggered, False otherwise
        """
        if not self.is_playing:
            logger.warning("Not playing")
            return False
            
        if not self.is_paused:
            logger.warning("Burst only works when paused")
            return False
        
        if seconds <= 0:
            logger.warning("Burst seconds must be positive")
            return False
            
        # Set burst parameters and trigger burst
        self.burst_seconds = seconds
        self.burst_requested.set()
        logger.info(f"Burst requested for {seconds} seconds")
        return True
    
    def play_next(self):
        """
        Play a single next message from the bag.
        Only works while paused.
        
        Returns:
            bool: True if message was played, False otherwise
        """
        if not self.is_playing:
            logger.warning("Not playing")
            return False
            
        if not self.is_paused:
            logger.warning("Play next only works when paused")
            return False
            
        # Trigger play next
        self.play_next_requested.set()
        return True
    
    def seek(self, timestamp: int):
        """
        Seek to a specific timestamp.
        
        Args:
            timestamp: Timestamp in nanoseconds
            
        Returns:
            bool: True if seek was successful, False otherwise
        """
        if not self.is_playing:
            logger.warning("Not playing")
            return False
        
        logger.info(f"Seeking to timestamp {timestamp}")
        
        # Store original pause state and pause playback
        was_paused = self.is_paused
        self.is_paused = True
        
        # Clear publish queue
        with self.queue_lock:
            self.publish_queue = []
        
        # Reset counters
        self.messages_played = 0
        self.played_since_last_report = 0
        
        # Update effective start time
        self.effective_start_time = timestamp
        
        try:
            # Open the MCAP file to find the proper position to start loading from
            with open(self.bag_path, 'rb') as f:
                reader = make_mcap_reader(f)
                
                # Find appropriate starting point for given timestamp
                start_index = 0
                found_target = False
                
                for i, (schema, channel, message) in enumerate(reader.iter_messages()):
                    # Apply topic filter
                    topic = channel.topic
                    if self.topics and topic not in self.topics:
                        continue
                        
                    # Check if this message is at or after our target timestamp
                    if message.log_time >= timestamp:
                        start_index = max(0, i - 10)  # Go back a few messages to ensure we don't miss anything
                        found_target = True
                        logger.debug(f"Found target timestamp at index {i}, starting at {start_index}")
                        break
                
                if not found_target:
                    # If we didn't find a message after the target timestamp, start from the end
                    logger.warning(f"Target timestamp {timestamp} is beyond end of file, seeking to end")
        except Exception as e:
            logger.error(f"Error finding timestamp position: {e}")
            # Restore original pause state
            self.is_paused = was_paused
            return False
        
        # Reload messages at new position
        # Load chunk from the appropriate position
        chunk, next_index = self._load_messages_chunk(start_index, self.chunk_size)
        if chunk:
            self.message_queue = chunk
            self.message_count = len(chunk)
            
            # Find message closest to requested timestamp in the loaded chunk
            self.current_index = 0
            for i, msg in enumerate(self.message_queue):
                if msg['timestamp'] >= timestamp:
                    self.current_index = i
                    break
            
            # Update reference times for playback timing
            ref_msg = self.message_queue[self.current_index]
            ref_bag_time = ref_msg['timestamp'] / 1e9
            ref_real_time = time.time()
            
            logger.info(f"Seeked to {self.current_index}/{self.message_count}")
            
            # Restore original pause state
            self.is_paused = was_paused
            return True
        else:
            logger.warning("No messages found after seek position")
            # Restore original pause state
            self.is_paused = was_paused
            return False


class LCMBagInfo:
    """Provides information about an MCAP file with LCM messages."""
    
    def __init__(self, bag_path: str):
        """
        Initialize the info provider.
        
        Args:
            bag_path: Path to the bag file to analyze
        """
        self.bag_path = bag_path
        self.topic_info = {}
        self.start_time = None
        self.end_time = None
        self.message_count = 0
        self.file_size = 0
        
        self._load_info()
    
    def _load_info(self):
        """Load information from the MCAP file."""
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Bag file not found: {self.bag_path}")
        
        self.file_size = os.path.getsize(self.bag_path)
        
        # Open the MCAP file
        with open(self.bag_path, 'rb') as f:
            reader = make_mcap_reader(f)
            
            # In the newer MCAP API, iter_messages() returns tuples of (schema, channel, message)
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                
                # Initialize topic info if not seen before
                if topic not in self.topic_info:
                    self.topic_info[topic] = {
                        'type': schema.name if schema else 'unknown',
                        'count': 0,
                        'encoding': channel.message_encoding
                    }
                
                # Update topic count
                self.topic_info[topic]['count'] += 1
                
                # Update time range
                if self.start_time is None or message.log_time < self.start_time:
                    self.start_time = message.log_time
                
                if self.end_time is None or message.log_time > self.end_time:
                    self.end_time = message.log_time
                
                self.message_count += 1
    
    def print_info(self):
        """Print information about the bag file."""
        print(f"File:             {self.bag_path}")
        print(f"Bag size:         {self._format_size(self.file_size)}")
        print(f"Storage id:       mcap")
        
        if self.start_time and self.end_time:
            duration_s = (self.end_time - self.start_time) / 1e9
            start_dt = datetime.datetime.fromtimestamp(self.start_time / 1e9)
            end_dt = datetime.datetime.fromtimestamp(self.end_time / 1e9)
            
            print(f"Duration:         {duration_s:.3f}s")
            print(f"Start:            {start_dt.strftime('%b %d %Y %H:%M:%S.%f')[:-3]} ({self.start_time // 1000000})")
            print(f"End               {end_dt.strftime('%b %d %Y %H:%M:%S.%f')[:-3]} ({self.end_time // 1000000})")
        
        print(f"Messages:         {self.message_count}")
        print("Topic information:", end="")
        
        if not self.topic_info:
            print(" No topics")
            return
        
        for i, (topic, info) in enumerate(self.topic_info.items()):
            prefix = " " if i == 0 else "                   "
            print(f"{prefix}Topic: {topic} | Type: {info['type']} | Count: {info['count']} | Serialization Format: {info['encoding']}")
    
    def _format_size(self, size_bytes):
        """Format size in bytes to human-readable format."""
        if size_bytes < 1024:
            return f"{size_bytes} B"
        elif size_bytes < 1024 * 1024:
            return f"{size_bytes / 1024:.1f} KiB"
        elif size_bytes < 1024 * 1024 * 1024:
            return f"{size_bytes / (1024 * 1024):.1f} MiB"
        else:
            return f"{size_bytes / (1024 * 1024 * 1024):.1f} GiB"


def list_topics(bag_path: str):
    """List topics in a bag file."""
    try:
        bag_info = LCMBagInfo(bag_path)
        print(f"Topics in {bag_path}:")
        if not bag_info.topic_info:
            print("  No topics found")
            return
            
        for topic, topic_info in bag_info.topic_info.items():
            print(f"  {topic} ({topic_info['type']}) - {topic_info['count']} messages")
    except Exception as e:
        logger.error(f"Error listing topics: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        sys.exit(1)


def show_info(bag_path: str):
    """Show information about a bag file."""
    try:
        bag_info = LCMBagInfo(bag_path)
        bag_info.print_info()
    except Exception as e:
        logger.error(f"Error showing bag info: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        sys.exit(1)


def setup_keyboard_control():
    """
    Set up keyboard control for interactive commands.
    
    Returns:
        A function that reads a single keystroke without requiring Enter
    """
    try:
        # Try to set up non-blocking keyboard input
        import termios
        import fcntl
        import sys
        import os
        
        # Save terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        
        def getch():
            try:
                # Set terminal to raw mode
                tty_settings = termios.tcgetattr(fd)
                tty_settings[3] = tty_settings[3] & ~termios.ICANON & ~termios.ECHO
                termios.tcsetattr(fd, termios.TCSANOW, tty_settings)
                
                # Set non-blocking
                fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)
                
                try:
                    # Try to read
                    ch = sys.stdin.read(1)
                    # Validate that we got a real character
                    if ch == '' or ch is None:
                        return None
                    return ch
                except (IOError, OSError):
                    # No data available
                    return None
            finally:
                # Restore terminal settings
                termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)
                fcntl.fcntl(fd, fcntl.F_SETFL, old_flags)
        
        return getch
        
    except (ImportError, AttributeError):
        # Fallback for non-Unix platforms
        try:
            import msvcrt
            
            def getch():
                if msvcrt.kbhit():
                    try:
                        ch = msvcrt.getch().decode('utf-8')
                        if ch is None or ch == '':
                            return None
                        return ch
                    except (UnicodeDecodeError, IOError):
                        return None
                return None
                
            return getch
            
        except ImportError:
            # Last resort - this won't be non-blocking
            def getch():
                return None
                
            return getch

def print_keyboard_controls(mode="player", snapshot_mode=False):
    """
    Print available keyboard controls.
    
    Args:
        mode: Either "player" or "recorder"
        snapshot_mode: Whether recorder is in snapshot mode
    """
    print("\nKeyboard Controls:")
    
    if mode == "recorder":
        if snapshot_mode:
            print("  n     - Take snapshot (save current buffer to disk)")
        
        print("  SPACE - Pause/Resume (standard pause - keeps counting time)")
        print("  f     - Freeze/Resume (time-freeze pause - stops time counting)")
        print("  t     - Toggle between pause modes")
    else:  # player mode
        print("  SPACE - Pause/Resume")
        print("  n     - Play next message (when paused)")
        print("  b     - Burst playback by 1 second (when paused)")
        print("  B     - Burst playback by 5 seconds (when paused)")
        print("  . (period) - Speed up playback")
        print("  , (comma)  - Slow down playback")
        print("  r     - Reset playback speed to 1x")
        print("  0-9   - Seek to percentage of bag (0=start, 9=90%, 5=50%)")
        print("  [     - Seek backwards 5 seconds")
        print("  ]     - Seek forwards 5 seconds")
        
    print("  q     - Quit")
    print("  s     - Show statistics")
    print("  ?     - Show this help")

def record(args):
    """Record LCM messages to a bag file."""
    # Set output path
    output_path = args.output or DEFAULT_BAG_NAME
    if not output_path.endswith('.mcap'):
        output_path += '.mcap'
    
    # Use the global recorder instance that was created in main()
    global _global_recorder
    recorder = _global_recorder
    
    # Set recorder mode/style based on arguments
    if args.snapshot_mode:
        logger.info("Recording in snapshot mode - use 'n' key or 'lcmbag.py snapshot' command to save buffer")
    
    # Register signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Stopping recording...")
        recorder.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Set up keyboard control if interactive mode is enabled
    if args.interactive:
        getch = setup_keyboard_control()
        print_keyboard_controls(mode="recorder", snapshot_mode=args.snapshot_mode)
    else:
        getch = None
    
    # Start recording
    try:
        recorder.start()
        
        # Keep main thread alive and handle keyboard input
        while recorder.is_recording:
            # Check for keyboard input in interactive mode
            key = None
            if getch and args.interactive:
                try:
                    key = getch()
                    # Safety check for key - make sure it's a valid character
                    if key is None or key == '':
                        key = None
                except Exception as e:
                    logger.debug(f"Error getting keyboard input: {e}")
                    key = None
                if key == ' ':  # spacebar - standard pause
                    if recorder.pause_mode != recorder.PAUSE_NONE:
                        recorder.resume()
                        print("Recording resumed")
                    else:
                        recorder.pause(freeze_time=False)
                        print("Recording paused (standard mode - time keeps counting)")
                elif key == 'f':  # 'f' key - freeze-time pause
                    if recorder.pause_mode != recorder.PAUSE_NONE:
                        recorder.resume()
                        print("Recording resumed")
                    else:
                        recorder.pause(freeze_time=True)
                        print("Recording paused (time-freeze mode - elapsed time paused)")
                elif key == 't':  # 't' key - toggle pause modes
                    if recorder.pause_mode == recorder.PAUSE_NONE:
                        # Not paused, so do nothing
                        print("Not paused - press SPACE or F to pause first")
                    elif recorder.pause_mode == recorder.PAUSE_SOFT:
                        # Switch from soft to freeze pause
                        recorder.pause(freeze_time=True)
                        print("Switched to time-freeze pause mode")
                    elif recorder.pause_mode == recorder.PAUSE_FREEZE:
                        # Switch from freeze to soft pause
                        recorder.pause(freeze_time=False)
                        print("Switched to standard pause mode")
                elif key == 'n' and args.snapshot_mode:  # 'n' key - take snapshot
                    # Trigger a snapshot - but only if key wasn't already processed
                    # Add a simple debounce to prevent multiple snapshots from a single keypress
                    # This is needed because terminal keystroke detection can sometimes register 
                    # the same key multiple times rapidly
                    if not hasattr(recorder, '_last_snapshot_time') or time.time() - recorder._last_snapshot_time > 1.0:
                        recorder._last_snapshot_time = time.time()
                        if recorder.trigger_snapshot():
                            print("Snapshot triggered - saving buffered messages to disk")
                        else:
                            print("Failed to trigger snapshot")
                elif key == 'q':
                    print("Quitting...")
                    break
                elif key == 's':
                    # Show statistics with new time tracking
                    elapsed = recorder.get_elapsed_time()
                    total_time = time.time() - recorder.start_time
                    
                    # Get pause status description
                    if recorder.pause_mode == recorder.PAUSE_NONE:
                        status = "Recording"
                    elif recorder.pause_mode == recorder.PAUSE_SOFT:
                        status = "Paused (standard mode)"
                    else:  # PAUSE_FREEZE
                        status = "Paused (time-freeze mode)"
                        
                    print(f"\nRecording Statistics:")
                    print(f"  Messages recorded: {recorder.message_count}")
                    print(f"  Messages dropped: {recorder.dropped_messages}")
                    print(f"  Total time: {total_time:.2f} seconds")
                    print(f"  Elapsed time: {elapsed:.2f} seconds")
                    if recorder.total_pause_duration > 0:
                        current_pause = 0
                        if recorder.pause_mode == recorder.PAUSE_FREEZE and recorder.pause_start_time > 0:
                            current_pause = time.time() - recorder.pause_start_time
                        print(f"  Time paused: {recorder.total_pause_duration + current_pause:.2f} seconds")
                    print(f"  Average rate: {recorder.message_count / elapsed:.1f} msgs/sec" if elapsed > 0 else "  Average rate: N/A")
                    print(f"  Current file: {recorder._get_chunk_filename()}")
                    print(f"  Status: {status}")
                    
                    # Show per-topic statistics if available
                    if hasattr(recorder, 'message_rates') and recorder.message_rates:
                        print("\nPer-topic statistics:")
                        current_time = time.time()
                        for channel, stats in recorder.message_rates.items():
                            if stats.get('count', 0) > 0:
                                channel_time_diff = current_time - stats.get('last_time', current_time)
                                if channel_time_diff > 0:
                                    rate = stats.get('count', 0) / channel_time_diff
                                    print(f"  {channel}: {rate:.1f} msgs/sec")
                elif key == '?':
                    print_keyboard_controls(mode="recorder")
            
            time.sleep(0.01)  # Short sleep to prevent CPU hogging
            
    except KeyboardInterrupt:
        logger.info("Recording interrupted")
    finally:
        recorder.stop()


def play(args):
    """Play back messages from a bag file."""
    # Convert time arguments from seconds to nanoseconds if provided
    start_time_ns = int(float(args.start) * 1e9) if args.start is not None else None
    end_time_ns = int(float(args.end) * 1e9) if args.end is not None else None
    
    # Create player with advanced options
    player = LCMBagPlayer(
        bag_path=args.bag_file,
        lcm_url=args.lcm_url,
        topics=args.topics,
        rate=args.rate,
        start_time=start_time_ns,
        end_time=end_time_ns,
        loop=args.loop,
        chunk_size=args.chunk_size
    )
    
    # Register signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Stopping playback...")
        player.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Set up keyboard control if interactive mode is enabled
    if args.interactive:
        getch = setup_keyboard_control()
        print_keyboard_controls(mode="player")
    else:
        getch = None
    
    # Start playback
    try:
        player.start()
        current_rate = args.rate
        
        # Make sure the timestamps and messages loaded correctly
        if not hasattr(player, 'first_timestamp') or player.first_timestamp is None:
            logger.warning("No timestamps could be loaded from the bag file")
        elif player.first_timestamp and player.last_timestamp:
            duration_s = (player.last_timestamp - player.first_timestamp) / 1e9
            logger.info(f"Bag duration: {duration_s:.2f} seconds")
            
        # Handle start in paused state if requested
        if args.pause:
            player.pause()
            print("Playback started in paused state")
            
            # If burst was specified, perform the burst and exit
            if args.burst is not None and args.burst > 0:
                if player.burst(args.burst):
                    print(f"Performed burst of {args.burst} seconds, exiting")
                    player.stop()
                    return
                else:
                    print("Failed to burst, continuing in paused state")
        
        # Keep main thread alive and handle keyboard input
        while player.is_playing:
            # Check for keyboard input in interactive mode
            key = None
            if getch and args.interactive:
                try:
                    key = getch()
                    # Safety check for key - make sure it's a valid character
                    if key is None or key == '':
                        key = None
                except Exception as e:
                    logger.debug(f"Error getting keyboard input: {e}")
                    key = None
                if key == ' ':  # spacebar
                    if player.is_paused:
                        player.resume()
                        print("Playback resumed")
                    else:
                        player.pause()
                        print("Playback paused")
                elif key == 'n' and player.is_paused:  # Play next message (only when paused)
                    if player.play_next():
                        print("Playing next message")
                    else:
                        print("Failed to play next message")
                elif key == 'b' and player.is_paused:  # Burst by 1 second (only when paused)
                    if player.burst(1.0):
                        print("Bursting playback by 1 second")
                    else:
                        print("Failed to burst playback")
                elif key == 'B' and player.is_paused:  # Burst by 5 seconds (only when paused)
                    if player.burst(5.0):
                        print("Bursting playback by 5 seconds")
                    else:
                        print("Failed to burst playback")
                elif key == 'q':
                    print("Quitting...")
                    break
                elif key == 's':
                    # Show statistics
                    elapsed = time.time() - player.start_play_time if player.start_play_time > 0 else 0
                    print(f"\nPlayback Statistics:")
                    print(f"  Messages played: {player.messages_played}")
                    print(f"  Elapsed time: {elapsed:.2f} seconds")
                    print(f"  Playback rate: {current_rate:.2f}x")
                    print(f"  Average publish rate: {player.messages_played / elapsed:.1f} msgs/sec" if elapsed > 0 else "  Average rate: N/A")
                    print(f"  Status: {'Paused' if player.is_paused else 'Playing'}")
                elif key == '.':  # Speed up
                    current_rate *= 1.5
                    print(f"Speed increased to {current_rate:.2f}x")
                    # Update player rate - note this is a simplification and won't affect already queued messages
                    player.rate = current_rate
                elif key == ',':  # Slow down
                    current_rate /= 1.5
                    print(f"Speed decreased to {current_rate:.2f}x")
                    # Update player rate - note this is a simplification and won't affect already queued messages
                    player.rate = current_rate
                elif key == 'r':  # Reset speed
                    current_rate = 1.0
                    print(f"Speed reset to {current_rate:.2f}x")
                    # Update player rate - note this is a simplification and won't affect already queued messages
                    player.rate = current_rate
                elif key and key == '[':  # Seek backward 5 seconds
                    if player.first_timestamp is not None and player.last_timestamp is not None:
                        try:
                            # Calculate current relative position based on our current index
                            # This is approximate but should work for basic controls
                            if player.current_index < len(player.message_queue):
                                current_msg = player.message_queue[player.current_index]
                                current_timestamp = current_msg['timestamp']
                                # Calculate 5 seconds earlier
                                seek_timestamp = max(player.first_timestamp, current_timestamp - 5_000_000_000)  # 5s in ns
                                
                                if player.seek(seek_timestamp):
                                    print(f"Seeking backward 5 seconds")
                                else:
                                    print("Failed to seek backward")
                            else:
                                print("Cannot determine current position")
                        except Exception as e:
                            logger.error(f"Error seeking backward: {e}")
                    else:
                        print("Seek information not available")
                elif key and key == ']':  # Seek forward 5 seconds
                    if player.first_timestamp is not None and player.last_timestamp is not None:
                        try:
                            # Calculate current relative position
                            if player.current_index < len(player.message_queue):
                                current_msg = player.message_queue[player.current_index]
                                current_timestamp = current_msg['timestamp']
                                # Calculate 5 seconds later
                                seek_timestamp = min(player.last_timestamp, current_timestamp + 5_000_000_000)  # 5s in ns
                                
                                if player.seek(seek_timestamp):
                                    print(f"Seeking forward 5 seconds")
                                else:
                                    print("Failed to seek forward")
                            else:
                                print("Cannot determine current position")
                        except Exception as e:
                            logger.error(f"Error seeking forward: {e}")
                    else:
                        print("Seek information not available")
                elif key and key in '0123456789':  # Seek to percentage
                    if player.first_timestamp is not None and player.last_timestamp is not None:
                        try:
                            # Calculate position based on percentage
                            percentage = int(key) * 10  # 0->0%, 1->10%, 9->90%
                            duration = player.last_timestamp - player.first_timestamp
                            seek_timestamp = player.first_timestamp + (duration * percentage // 100)
                            
                            if player.seek(seek_timestamp):
                                print(f"Seeking to {percentage}% of bag file")
                            else:
                                print(f"Failed to seek to {percentage}%")
                        except (ValueError, TypeError) as e:
                            # Handle case if key is not a valid digit
                            logger.error(f"Error seeking with key '{key}': {e}")
                    else:
                        print("Seek information not available")
                elif key == '?':
                    print_keyboard_controls(mode="player")
            
            time.sleep(0.01)  # Short sleep to prevent CPU hogging
            
    except KeyboardInterrupt:
        logger.info("Playback interrupted")
    finally:
        player.stop()


# Global recorder instance for snapshot command
_global_recorder = None

def trigger_snapshot_command():
    """Command to trigger a snapshot for a running recorder in snapshot mode."""
    global _global_recorder
    
    if _global_recorder is None or not _global_recorder.is_recording:
        logger.error("No active recorder found - start a recorder with --snapshot-mode first")
        return False
        
    if _global_recorder.recording_mode != _global_recorder.MODE_SNAPSHOT:
        logger.error("Recorder is not in snapshot mode")
        return False
        
    # Trigger snapshot
    return _global_recorder.trigger_snapshot()

def main():
    """Main entry point."""
    # Configure logger to avoid duplicate entries
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    parser = argparse.ArgumentParser(description='LCM Bag - Record and play back LCM messages')
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Record command with advanced options
    record_parser = subparsers.add_parser('record', help='Record LCM messages to a bag file')
    record_parser.add_argument('-o', '--output', help='Output bag file path')
    record_parser.add_argument('-u', '--lcm-url', default=DEFAULT_LCM_URL, help='LCM URL to subscribe to')
    record_parser.add_argument('-i', '--interactive', action='store_true',
                              help='Enable interactive keyboard controls for pause/resume/stats')
    record_parser.add_argument('--snapshot-mode', action='store_true',
                              help='Enable snapshot mode - buffer messages in memory until snapshot is triggered')
    record_parser.add_argument('--max-cache-size', type=int, default=1000000,
                              help='Maximum number of messages to keep in snapshot circular buffer (default: 1000000)')
    record_parser.add_argument('--max-size', type=int, default=0, 
                              help='Maximum size of each file chunk in MB (0 for unlimited)')
    record_parser.add_argument('--buffer-size', type=int, default=10000,
                              help='Maximum number of messages to buffer before writing (default: 10000)')
    record_parser.add_argument('--stats-interval', type=int, default=1000,
                              help='Number of messages between statistics logging (default: 1000)')
    record_parser.add_argument('topics', nargs='*', help='Topics to record (empty for all topics)')
    
    # Snapshot command - trigger a snapshot for a running recorder
    snapshot_parser = subparsers.add_parser('snapshot', 
                      help='Trigger a snapshot for a running recorder in snapshot mode')
    
    # Play command with advanced options
    play_parser = subparsers.add_parser('play', help='Play back messages from a bag file')
    play_parser.add_argument('bag_file', help='Bag file to play')
    play_parser.add_argument('-r', '--rate', type=float, default=1.0, 
                            help='Playback rate multiplier (default: 1.0)')
    play_parser.add_argument('-u', '--lcm-url', default=DEFAULT_LCM_URL, 
                            help='LCM URL to publish to')
    play_parser.add_argument('-i', '--interactive', action='store_true',
                            help='Enable interactive keyboard controls for pause/resume/speed/stats')
    play_parser.add_argument('--start', type=float, 
                            help='Start playback from this timestamp in seconds')
    play_parser.add_argument('--end', type=float, 
                            help='End playback at this timestamp in seconds')
    play_parser.add_argument('--loop', action='store_true', 
                            help='Loop playback when reaching the end')
    play_parser.add_argument('--chunk-size', type=int, default=1000,
                            help='Number of messages to load at once (default: 1000)')
    play_parser.add_argument('--topics', nargs='+', 
                            help='Topics to play (empty for all topics)')
    play_parser.add_argument('--burst', type=float,
                            help='Burst playback by specified seconds and exit (requires --pause)')
    play_parser.add_argument('--pause', action='store_true',
                            help='Start playback in paused state')
    
    # Info command
    info_parser = subparsers.add_parser('info', help='Display information about a bag file')
    info_parser.add_argument('bag_file', help='Bag file to show info for')
    
    # List command
    list_parser = subparsers.add_parser('list', help='List available topics in a bag file')
    list_parser.add_argument('bag_file', help='Bag file to list topics from')
    
    # Parse arguments
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # Execute command
    if args.command == 'record':
        # Store recorder globally for snapshot command
        global _global_recorder
        
        # Clean up any existing recorder
        if _global_recorder is not None:
            try:
                _global_recorder.stop()
            except:
                pass
        
        # Set output path
        output_path = args.output or DEFAULT_BAG_NAME
        if not output_path.endswith('.mcap'):
            output_path += '.mcap'
        
        # Create new recorder
        _global_recorder = LCMBagRecorder(
            output_path=output_path,
            lcm_url=args.lcm_url,
            topics=args.topics,
            max_file_size_mb=args.max_size,
            buffer_capacity=args.buffer_size,
            stats_interval=args.stats_interval,
            snapshot_mode=args.snapshot_mode,
            max_cache_size=args.max_cache_size
        )
        record(args)
    elif args.command == 'snapshot':
        success = trigger_snapshot_command()
        if success:
            print("Snapshot triggered successfully")
        else:
            print("Failed to trigger snapshot")
            sys.exit(1)
    elif args.command == 'play':
        play(args)
    elif args.command == 'info':
        show_info(args.bag_file)
    elif args.command == 'list':
        list_topics(args.bag_file)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import numpy as np
import lcm
import threading
import time
import cv2
import struct
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from lcm_msgs.sensor_msgs import Image, CameraInfo, PointCloud2, PointField
from lcm_msgs.std_msgs import Header

class DepthToPointcloudConverter:
    def __init__(self, swap_y_z=False, colorize=True):
        self.lc = lcm.LCM()
        self.lc_thread = None
        self.running = True
        
        # Initialize last received messages
        self.last_depth_image = None
        self.last_depth_stamp = 0
        self.last_rgb_image = None
        self.last_rgb_stamp = 0
        self.camera_info = None
        self.camera_info_received = False
        
        # Cache for optimization
        self.cached_x_map = None
        self.cached_y_map = None
        self.cached_dimensions = None
        self.cache_lock = threading.RLock()  # Use reentrant lock for thread safety
        
        # Multithreading
        self.pool = ThreadPoolExecutor(max_workers=4)  # Adjust as needed
        
        # Optimization settings
        self.downsample_factor = 1  # Set to higher for downsampling (e.g., 2 for half resolution)
        self.filter_threshold = 0.1  # Filter out points closer than this value (in meters)
        self.max_depth = 10.0  # Filter out points further than this value (in meters)
        
        # Coordinate system settings
        self.swap_y_z = swap_y_z  # If True, will swap Y and Z axes
        
        # Colorization settings
        self.colorize = colorize
        self.max_time_diff = 0.5  # Maximum time difference between depth and RGB (in seconds)
        
        print(f"Point cloud axes: {'Y and Z axes swapped' if self.swap_y_z else 'Standard XYZ mapping'}")
        print(f"Colorization: {'Enabled' if self.colorize else 'Disabled'}")
        
        # Subscribe to topics
        self.lc.subscribe("head_cam_depth#sensor_msgs.Image", self.depth_callback)
        self.lc.subscribe("head_cam_depth_info#sensor_msgs.CameraInfo", self.camera_info_callback)
        
        # Subscribe to RGB image if colorize is enabled
        if self.colorize:
            self.lc.subscribe("head_cam_rgb#sensor_msgs.Image", self.rgb_callback)
        
        # Start LCM thread
        self.start_lcm_thread()
        
    def depth_callback(self, channel, data):
        try:
            # Decode LCM message
            img_msg = Image.decode(data)
            
            # Convert image data to numpy array
            if img_msg.encoding == "32FC1":
                # float32 depth in meters
                depth_img = np.frombuffer(img_msg.data, dtype=np.float32)
                depth_img = depth_img.reshape((img_msg.height, img_msg.width))
            elif img_msg.encoding == "mono16":
                # uint16 depth in millimeters
                depth_img = np.frombuffer(img_msg.data, dtype=np.uint16)
                depth_img = depth_img.reshape((img_msg.height, img_msg.width)) / 1000.0  # Convert to meters
            else:
                print(f"Unsupported depth encoding: {img_msg.encoding}")
                return
            
            # Cache the image and timestamp
            self.last_depth_image = depth_img
            
            # Convert LCM timestamp to seconds
            if hasattr(img_msg.header.stamp, 'sec') and hasattr(img_msg.header.stamp, 'nsec'):
                self.last_depth_stamp = img_msg.header.stamp.sec + img_msg.header.stamp.nsec * 1e-9
            else:
                self.last_depth_stamp = time.time()
            
            # If we have camera info, convert depth to point cloud in a separate thread
            if self.camera_info_received:
                # If colorization is enabled, make sure we also have the RGB image
                if self.colorize and self.last_rgb_image is not None:
                    time_diff = abs(self.last_depth_stamp - self.last_rgb_stamp)
                    if time_diff <= self.max_time_diff:
                        # We have synchronized RGB and depth, process together
                        self.pool.submit(self.process_depth_image, depth_img, self.last_rgb_image, img_msg.header)
                    else:
                        # Print warning about timestamp difference if it's too large
                        if time_diff > 1.0:  # Only warn for large differences
                            print(f"Warning: Large time difference between depth and RGB: {time_diff:.2f}s")
                        # Process depth only if RGB is too old
                        self.pool.submit(self.process_depth_image, depth_img, None, img_msg.header)
                else:
                    # Process depth without RGB
                    self.pool.submit(self.process_depth_image, depth_img, None, img_msg.header)
                
        except Exception as e:
            print(f"Error in depth callback: {e}")
            import traceback
            traceback.print_exc()
    
    def rgb_callback(self, channel, data):
        try:
            # Decode LCM message
            img_msg = Image.decode(data)
            
            # Handle different encodings
            if img_msg.encoding in ["rgb8", "bgr8"]:
                # RGB or BGR format
                rgb_img = np.frombuffer(img_msg.data, dtype=np.uint8)
                rgb_img = rgb_img.reshape((img_msg.height, img_msg.width, 3))
                
                # Convert BGR to RGB if needed
                if img_msg.encoding == "bgr8":
                    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
            else:
                print(f"Unsupported RGB encoding: {img_msg.encoding}")
                return
            
            # Cache the image and timestamp
            self.last_rgb_image = rgb_img
            
            # Convert LCM timestamp to seconds
            if hasattr(img_msg.header.stamp, 'sec') and hasattr(img_msg.header.stamp, 'nsec'):
                self.last_rgb_stamp = img_msg.header.stamp.sec + img_msg.header.stamp.nsec * 1e-9
            else:
                self.last_rgb_stamp = time.time()
                
        except Exception as e:
            print(f"Error in RGB callback: {e}")
            import traceback
            traceback.print_exc()
    
    def camera_info_callback(self, channel, data):
        try:
            # Decode LCM message
            new_camera_info = CameraInfo.decode(data)
            
            # Check if camera info has changed significantly
            needs_cache_update = True
            if self.camera_info is not None:
                # Check if key parameters are the same
                if (self.camera_info.width == new_camera_info.width and
                    self.camera_info.height == new_camera_info.height and
                    self.camera_info.K[0] == new_camera_info.K[0] and
                    self.camera_info.K[4] == new_camera_info.K[4] and
                    self.camera_info.K[2] == new_camera_info.K[2] and
                    self.camera_info.K[5] == new_camera_info.K[5]):
                    needs_cache_update = False
            
            # Update the camera info
            self.camera_info = new_camera_info
            
            # Mark that we've received the camera info
            self.camera_info_received = True
            
            # Clear the cache if needed
            if needs_cache_update:
                with self.cache_lock:
                    self.cached_x_map = None
                    self.cached_y_map = None
                    self.cached_dimensions = None
                    print(f"Received new camera info: f={self.camera_info.K[0]:.1f}, width={self.camera_info.width}, height={self.camera_info.height}")
            
        except Exception as e:
            print(f"Error in camera info callback: {e}")
            import traceback
            traceback.print_exc()
    
    def lcm_thread_func(self):
        """Thread function to handle LCM messages in the background"""
        while self.running:
            try:
                self.lc.handle_timeout(10)  # 10ms timeout
            except Exception as e:
                print(f"LCM handling error: {e}")
                time.sleep(0.001)  # Prevent CPU overuse on errors
    
    def start_lcm_thread(self):
        """Start the LCM handling thread"""
        self.lcm_thread = threading.Thread(target=self.lcm_thread_func)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
    
    def create_point_cloud2_msg(self, points, colors=None, header=None):
        """
        Create a PointCloud2 message from point data
        :param points: Nx3 numpy array of (x, y, z) points
        :param colors: Nx3 numpy array of (r, g, b) colors (optional)
        :param header: Header to use for the message (optional)
        """
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        
        # Set header
        if header is not None:
            cloud_msg.header = header
        else:
            cloud_msg.header = Header()
            cloud_msg.header.stamp = int(time.time() * 1e9)  # Current time in nanoseconds
        
        cloud_msg.header.frame_id = "pan_tilt_head"
        
        # Set basic properties
        N = points.shape[0]
        cloud_msg.height = 1
        cloud_msg.width = N
        
        # Define datatype constants
        # INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8
        FLOAT32 = 7
        UINT8 = 2
        UINT32 = 6
        
        # Define point structure
        cloud_msg.fields = []
        
        # Define XYZ fields
        field_x = PointField()
        field_x.name = "x"
        field_x.offset = 0
        field_x.datatype = FLOAT32
        field_x.count = 1
        cloud_msg.fields.append(field_x)
        
        field_y = PointField()
        field_y.name = "y"
        field_y.offset = 4
        field_y.datatype = FLOAT32
        field_y.count = 1
        cloud_msg.fields.append(field_y)
        
        field_z = PointField()
        field_z.name = "z"
        field_z.offset = 8
        field_z.datatype = FLOAT32
        field_z.count = 1
        cloud_msg.fields.append(field_z)
        
        # Define color fields if colors are provided
        if colors is not None:
            # Add a single RGBA field (uint32 packed color format for Foxglove)
            field_rgba = PointField()
            field_rgba.name = "rgba"
            field_rgba.offset = 12
            field_rgba.datatype = UINT32
            field_rgba.count = 1
            cloud_msg.fields.append(field_rgba)
            
            point_step = 16  # 16 bytes per point: 12 bytes for XYZ (3 float32) + 4 bytes for RGBA
        else:
            # If no colors, add intensity field for compatibility
            field_intensity = PointField()
            field_intensity.name = "intensity"
            field_intensity.offset = 12
            field_intensity.datatype = FLOAT32
            field_intensity.count = 1
            cloud_msg.fields.append(field_intensity)
            
            point_step = 16  # 16 bytes per point: 12 bytes for XYZ (3 float32) + 4 bytes for intensity
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = point_step * N
        
        # Create data array
        if colors is not None:
            # Convert RGB values to packed RGBA uint32 format
            # RGBA = (R << 24) | (G << 16) | (B << 8) | A
            # In little-endian format, it's stored as ABGR in memory
            # Pack into single uint32 (RGBA format for Foxglove compatibility)
            rgba_packed = np.zeros(N, dtype=np.uint32)
            for i in range(N):
                r, g, b = colors[i]
                # Pack RGBA: (R << 24) | (G << 16) | (B << 8) | 0xFF (alpha=255)
                rgba_packed[i] = (int(r) << 24) | (int(g) << 16) | (int(b) << 8) | 0xFF
            
            # Create binary data
            points_float32 = points.astype(np.float32)
            
            # Prepare the data buffer
            cloud_data_bytes = bytearray(N * 16)
            
            # Copy XYZ data
            points_bytes = points_float32.tobytes()
            rgba_bytes = rgba_packed.tobytes()
            
            # Interleave the data
            for i in range(N):
                # Copy XYZ data (12 bytes)
                cloud_data_bytes[i*16:i*16+12] = points_bytes[i*12:i*12+12]
                # Copy RGBA data (4 bytes)
                cloud_data_bytes[i*16+12:i*16+16] = rgba_bytes[i*4:i*4+4]
            
            cloud_msg.data = bytes(cloud_data_bytes)
            
        else:
            # Create intensity column (all 1.0 for visibility)
            intensity = np.ones((N, 1), dtype=np.float32)
            
            # Combine XYZ with intensity field
            cloud_data = np.hstack([points.astype(np.float32), intensity])
            
            # Set data
            cloud_msg.data = cloud_data.tobytes()
        
        # Debug info to verify byte alignment
        data_size = len(cloud_msg.data)
        bytes_per_point = data_size / N if N > 0 else 0
        print(f"PointCloud: {N} points, {data_size} bytes total, {bytes_per_point:.1f} bytes per point")
        print(f"Fields: {[f.name for f in cloud_msg.fields]}, point_step: {cloud_msg.point_step}")
        
        # Final attributes
        cloud_msg.is_dense = True
        cloud_msg.data_length = len(cloud_msg.data)
        
        return cloud_msg
    
    def generate_point_cloud(self, depth_image, rgb_image=None):
        """
        Convert a depth image to a point cloud, optionally with color
        :param depth_image: numpy array with depth values
        :param rgb_image: numpy array with RGB values (optional)
        :return: tuple of (points_xyz, colors_rgb) or just points_xyz if no rgb_image
        """
        # Check if camera info is available
        if self.camera_info is None:
            print("Camera info not available")
            return None
            
        # Get the original dimensions of the depth image
        height, width = depth_image.shape
        
        # Determine if we need to create or update the lookup maps
        need_new_maps = False
        current_dims = None
        
        # Safely check the cache status using the lock
        with self.cache_lock:
            if (self.cached_x_map is None or self.cached_y_map is None or 
                self.cached_dimensions != (height, width)):
                need_new_maps = True
                
        # If we need new maps, create them
        if need_new_maps:
            # Get focal lengths and principal point from camera info
            # In the camera matrix: [fx, 0, cx; 0, fy, cy; 0, 0, 1]
            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            cx = self.camera_info.K[2]
            cy = self.camera_info.K[5]
            
            # Apply downsampling if requested
            if self.downsample_factor > 1:
                height = height // self.downsample_factor
                width = width // self.downsample_factor
                depth_image = cv2.resize(depth_image, (width, height), interpolation=cv2.INTER_NEAREST)
                
                # Also resize RGB image if provided
                if rgb_image is not None:
                    rgb_image = cv2.resize(rgb_image, (width, height), interpolation=cv2.INTER_LINEAR)
                
                fx = fx / self.downsample_factor
                fy = fy / self.downsample_factor
                cx = cx / self.downsample_factor
                cy = cy / self.downsample_factor
            
            # Create coordinate maps (these can be reused for all frames with the same dimensions)
            # Using meshgrid is more efficient than nested loops
            v, u = np.mgrid[:height, :width]
            
            # Pre-compute fixed parts of the projection - create local copies first
            x_map = (u - cx) / fx
            y_map = (v - cy) / fy
            dims = (height, width)
            
            # Now update the cached values safely
            with self.cache_lock:
                self.cached_x_map = x_map
                self.cached_y_map = y_map
                self.cached_dimensions = dims
                
            # Use the local copies we just created
            x_map_to_use = x_map
            y_map_to_use = y_map
            current_dims = dims
        else:
            # Use existing cache safely
            with self.cache_lock:
                # Make local copies of the cached data
                x_map_to_use = self.cached_x_map.copy() if self.cached_x_map is not None else None
                y_map_to_use = self.cached_y_map.copy() if self.cached_y_map is not None else None
                current_dims = self.cached_dimensions
                
            # Double check that our cache is valid
            if x_map_to_use is None or y_map_to_use is None or current_dims is None:
                print("Cache is invalid, skipping frame")
                return None
                
            # Apply downsampling if necessary
            if self.downsample_factor > 1:
                depth_image = cv2.resize(depth_image, 
                                        (current_dims[1], current_dims[0]), 
                                        interpolation=cv2.INTER_NEAREST)
                
                # Also resize RGB image if provided
                if rgb_image is not None:
                    rgb_image = cv2.resize(rgb_image, 
                                         (current_dims[1], current_dims[0]), 
                                         interpolation=cv2.INTER_LINEAR)
        
        # Apply filters to depth image
        valid_mask = np.logical_and(
            depth_image > self.filter_threshold,  # Remove points too close
            depth_image < self.max_depth          # Remove points too far
        )
        
        # Extract valid depth values and coordinates
        valid_depths = depth_image[valid_mask]
        
        # Use the local copies for thread safety
        valid_x_map = x_map_to_use[valid_mask]
        valid_y_map = y_map_to_use[valid_mask]
        
        # Compute 3D points
        points_x = valid_x_map * valid_depths
        points_y = valid_y_map * valid_depths
        points_z = valid_depths
        
        # Get RGB colors if available
        if rgb_image is not None:
            # Extract colors using the same valid mask
            # Note: need to get indices of the mask to index into rgb_image
            valid_indices = np.where(valid_mask)
            valid_colors = rgb_image[valid_indices]
            
            # Stack into arrays
            if self.swap_y_z:
                # Swap Y and Z axes for correct orientation in visualization
                points = np.column_stack((points_x, points_z, -points_y))  # Negate Y to flip axis
            else:
                # Use standard camera coordinate system 
                points = np.column_stack((points_x, points_y, points_z))
                
            return points, valid_colors
        else:
            # Return just points (no colors)
            if self.swap_y_z:
                # Swap Y and Z axes for correct orientation in visualization
                return np.column_stack((points_x, points_z, -points_y)), None  # Negate Y to flip axis
            else:
                # Use standard camera coordinate system 
                return np.column_stack((points_x, points_y, points_z)), None
    
    def process_depth_image(self, depth_image, rgb_image=None, header=None):
        """
        Process depth image and publish point cloud
        :param depth_image: numpy array with depth values
        :param rgb_image: numpy array with RGB values (optional)
        :param header: header from the original depth image message
        """
        try:
            start_time = time.time()
            
            # Generate point cloud
            result = self.generate_point_cloud(depth_image, rgb_image)
            
            # Check if point cloud generation failed
            if result is None:
                print("Point cloud generation failed, skipping this frame")
                return
                
            # Unpack the results - could be (points, colors) or just points
            if isinstance(result, tuple) and len(result) == 2:
                points, colors = result
            else:
                print("Unexpected result format from generate_point_cloud")
                return
            
            # Verify we have valid points data
            if points is None:
                print("No points data available, skipping this frame")
                return
                
            if len(points) == 0:
                print("Empty point cloud, skipping this frame")
                return
            
            # Create PointCloud2 message with proper error checking
            try:
                cloud_msg = self.create_point_cloud2_msg(points, colors, header=header)
            except Exception as e:
                print(f"Error creating point cloud message: {e}")
                return
                
            # Publish the message
            try:
                self.lc.publish("head_cam_pointcloud#sensor_msgs.PointCloud2", cloud_msg.encode())
                
                # Debug output
                processing_time = time.time() - start_time
                print(f"Published {'colored ' if colors is not None else ''}pointcloud with {len(points)} points in {processing_time*1000:.1f}ms")
            except Exception as e:
                print(f"Error publishing point cloud: {e}")
                
        except Exception as e:
            print(f"Error processing depth image: {e}")
            import traceback
            traceback.print_exc()
            
    def stop(self):
        """Stop the LCM handling thread"""
        self.running = False
        if self.lcm_thread is not None:
            self.lcm_thread.join(timeout=1.0)
        self.pool.shutdown()

def main():
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Convert depth images to colored point clouds')
    parser.add_argument('--swap-y-z', action='store_true', help='Swap Y and Z axes for visualization')
    parser.add_argument('--downsample', type=int, default=1, help='Downsample factor (1=no downsampling, 2=half resolution, etc.)')
    parser.add_argument('--max-depth', type=float, default=10.0, help='Maximum depth to include in meters')
    parser.add_argument('--min-depth', type=float, default=0.1, help='Minimum depth to include in meters')
    parser.add_argument('--no-color', action='store_true', help='Disable colorization')
    args = parser.parse_args()
    
    # Initialize depth to pointcloud converter
    converter = DepthToPointcloudConverter(swap_y_z=args.swap_y_z, colorize=not args.no_color)
    
    # Set additional parameters
    converter.downsample_factor = args.downsample
    converter.max_depth = args.max_depth
    converter.filter_threshold = args.min_depth
    
    try:
        print("Depth to colored pointcloud converter running with settings:")
        print(f"  Swap Y-Z axes: {args.swap_y_z}")
        print(f"  Colorization: {'Disabled' if args.no_color else 'Enabled'}")
        print(f"  Downsample factor: {args.downsample}")
        print(f"  Depth range: {args.min_depth} to {args.max_depth} meters")
        print("Waiting for depth image, RGB image, and camera info messages...")
        
        # Keep the main thread alive
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down...")
    finally:
        # Clean up
        converter.stop()
        print("Converter stopped.")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import numpy as np
import lcm
import threading
import time
import cv2
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from lcm_msgs.sensor_msgs import Image, CameraInfo, PointCloud2, PointField
from lcm_msgs.std_msgs import Header

# Drake imports
from pydrake.systems.sensors import ImageDepth32F, CameraInfo as DrakeCameraInfo, PixelType
from pydrake.perception import DepthImageToPointCloud
from pydrake.systems.framework import Context, LeafSystem
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph

class DrakeDepthToPointcloudConverter:
    def __init__(self, swap_y_z=False):
        self.lc = lcm.LCM()
        self.lc_thread = None
        self.running = True
        
        # Initialize last received messages
        self.last_depth_image = None
        self.last_depth_stamp = 0
        self.camera_info = None
        self.camera_info_received = False
        self.frame_id = "camera"
        
        # Drake DepthImageToPointCloud system
        self.drake_camera_info = None
        self.depth_to_pc_system = None
        
        # Multithreading
        self.pool = ThreadPoolExecutor(max_workers=4)  # Adjust as needed
        
        # Optimization settings
        self.downsample_factor = 1  # Set to higher for downsampling (e.g., 2 for half resolution)
        self.filter_threshold = 0.1  # Filter out points closer than this value (in meters)
        self.max_depth = 10.0  # Filter out points further than this value (in meters)
        
        # Coordinate system settings
        self.swap_y_z = swap_y_z  # If True, will swap Y and Z axes
        print(f"Point cloud axes: {'Y and Z axes swapped' if self.swap_y_z else 'Standard XYZ mapping'}")
        
        # Subscribe to topics
        self.lc.subscribe("head_cam_depth#sensor_msgs.Image", self.depth_callback)
        self.lc.subscribe("head_cam_depth_info#sensor_msgs.CameraInfo", self.camera_info_callback)
        
        # Start LCM thread
        self.start_lcm_thread()
        
    def initialize_drake_system(self):
        """Initialize the Drake DepthImageToPointCloud system with the current camera info"""
        if self.camera_info is None:
            return False
        
        # Convert LCM CameraInfo to Drake CameraInfo
        # Drake CameraInfo requires: width, height, focal_x, focal_y, center_x, center_y
        width = self.camera_info.width
        height = self.camera_info.height
        fx = self.camera_info.K[0]  # Focal length x
        fy = self.camera_info.K[4]  # Focal length y
        cx = self.camera_info.K[2]  # Principal point x
        cy = self.camera_info.K[5]  # Principal point y
        
        # Apply downsampling if requested
        if self.downsample_factor > 1:
            width = width // self.downsample_factor
            height = height // self.downsample_factor
            fx = fx / self.downsample_factor
            fy = fy / self.downsample_factor
            cx = cx / self.downsample_factor
            cy = cy / self.downsample_factor
        
        # Create Drake CameraInfo
        self.drake_camera_info = DrakeCameraInfo(width=width, height=height, 
                                                focal_x=fx, focal_y=fy, 
                                                center_x=cx, center_y=cy)
        
        # Create DepthImageToPointCloud system
        # Using PixelType.kDepth32F since we'll convert all depths to float32
        self.depth_to_pc_system = DepthImageToPointCloud(
            camera_info=self.drake_camera_info,
            pixel_type=PixelType.kDepth32F,
            scale=1.0,  # We'll handle any scaling before passing to Drake
            fields=2  # BaseField.kXYZs - basic point cloud with XYZ coordinates
        )
        
        # Create a context specific to this system
        # Important: Every time we recreate the system, we need a new context
        self.context = self.depth_to_pc_system.CreateDefaultContext()
        
        print(f"Initialized Drake DepthImageToPointCloud with camera parameters:")
        print(f"  Size: {width}x{height}")
        print(f"  Focal: ({fx:.1f}, {fy:.1f})")
        print(f"  Center: ({cx:.1f}, {cy:.1f})")
        
        return True
        
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
            self.last_depth_stamp = img_msg.header.stamp
            self.frame_id = img_msg.header.frame_id
            
            # If we have camera info, convert depth to point cloud in a separate thread
            if self.camera_info_received:
                self.pool.submit(self.process_depth_image, depth_img, img_msg.header)
                
        except Exception as e:
            print(f"Error in depth callback: {e}")
            import traceback
            traceback.print_exc()
    
    def camera_info_callback(self, channel, data):
        try:
            # Decode LCM message
            self.camera_info = CameraInfo.decode(data)
            
            # Mark that we've received the camera info
            self.camera_info_received = True
            
            # Initialize Drake system with this camera info
            success = self.initialize_drake_system()
            
            if success:
                print(f"Received camera info: f={self.camera_info.K[0]:.1f}, width={self.camera_info.width}, height={self.camera_info.height}")
            
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
    
    def create_point_cloud2_msg(self, points, header=None):
        """
        Create a PointCloud2 message from point data
        :param points: Nx3 numpy array of (x, y, z) points
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
        
        cloud_msg.header.frame_id = self.frame_id
        
        # Set basic properties
        N = points.shape[0]
        cloud_msg.height = 1
        cloud_msg.width = N
        
        # Define datatype constants
        # INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8
        FLOAT32 = 7
        
        # Define point structure (always XYZ + intensity for simplicity)
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
        
        # Always add intensity field (critical for compatibility)
        field_intensity = PointField()
        field_intensity.name = "intensity"
        field_intensity.offset = 12
        field_intensity.datatype = FLOAT32
        field_intensity.count = 1
        cloud_msg.fields.append(field_intensity)
        
        # Fixed point step for all points (4 float32 values per point)
        point_step = 16
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = point_step * N
        
        # Create intensity column (all 1.0 for visibility)
        intensity = np.ones((N, 1), dtype=np.float32)
        
        # Combine XYZ with intensity field
        cloud_data = np.hstack([points.astype(np.float32), intensity])
        
        # Set data - must be exactly 16 bytes per point
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
    
    def drake_depth_to_pointcloud(self, depth_image):
        """
        Convert a depth image to a point cloud using Drake's DepthImageToPointCloud
        :param depth_image: numpy array with depth values (float32, in meters)
        :return: Nx3 numpy array of (x, y, z) points
        """
        if self.depth_to_pc_system is None:
            print("Drake DepthImageToPointCloud system not initialized")
            return None
        
        # Detailed timing for performance analysis
        start_total = time.time()
        
        # Apply downsampling if requested (this can significantly improve performance)
        start_downsample = time.time()
        if self.downsample_factor > 1:
            depth_image = cv2.resize(depth_image, 
                                    (self.drake_camera_info.width(), self.drake_camera_info.height()), 
                                    interpolation=cv2.INTER_NEAREST)
        downsample_time = time.time() - start_downsample
        
        # Apply filters to depth image before conversion
        # Performance optimization: Don't create a copy unless necessary
        start_filter = time.time()
        depth_image_filtered = depth_image  # Reference first
        
        # Only create a copy if filters are applied
        if self.filter_threshold > 0 or self.max_depth < float('inf'):
            depth_image_filtered = np.copy(depth_image)  
            
            # Apply depth range filtering - use vectorized operations for speed
            mask = (depth_image_filtered < self.filter_threshold) | (depth_image_filtered > self.max_depth)
            if mask.any():
                depth_image_filtered[mask] = np.nan
        filter_time = time.time() - start_filter
        
        # Drake processing
        start_drake = time.time()
        
        # Create Drake ImageDepth32F from numpy array
        start_image_create = time.time()
        drake_depth_image = ImageDepth32F(width=depth_image_filtered.shape[1], height=depth_image_filtered.shape[0])
        
        # Drake expects a 3D array with shape (height, width, 1)
        # Reshape the depth image to match the expected format
        # Performance optimization: Use reshape view when possible instead of copy
        reshaped_depth = depth_image_filtered.reshape(depth_image_filtered.shape[0], depth_image_filtered.shape[1], 1)
        drake_depth_image.mutable_data[:] = reshaped_depth
        image_create_time = time.time() - start_image_create
        
        # Set the depth image as an input to the system
        start_drake_process = time.time()
        self.depth_to_pc_system.depth_image_input_port().FixValue(
            self.context, drake_depth_image)
        
        # Get the output point cloud
        output_port = self.depth_to_pc_system.point_cloud_output_port()
        drake_point_cloud = output_port.Eval(self.context)
        drake_process_time = time.time() - start_drake_process
        
        # Convert Drake PointCloud to numpy array
        start_conversion = time.time()
        # Drake point cloud stores XYZ as floats in its 'xyzs' field
        # Performance optimization: Use direct array access when possible
        points_array = np.asarray(drake_point_cloud.xyzs()).T  # Shape (N, 3)
        conversion_time = time.time() - start_conversion
        
        # Filter invalid points
        start_filter_points = time.time()
        # Filtering is slow if there are many invalid points, only do if necessary
        if np.isnan(points_array).any() or np.isinf(points_array).any():
            valid_mask = ~np.any(np.isnan(points_array) | np.isinf(points_array), axis=1)
            points_array = points_array[valid_mask]
        filter_points_time = time.time() - start_filter_points
        
        drake_time = time.time() - start_drake
        
        # Apply coordinate transformation if needed
        start_swap = time.time()
        result = None
        if self.swap_y_z:
            # Swap Y and Z axes for visualization
            # Standard camera convention: X right, Y down, Z forward
            # After swap: X right, Z down (up becomes positive), Y forward (depth)
            result = np.column_stack((points_array[:, 0], points_array[:, 2], -points_array[:, 1]))
            if len(result) > 0:
                print(f"Y-Z axes swapped! Example point: [{result[0,0]:.2f}, {result[0,1]:.2f}, {result[0,2]:.2f}]")
        else:
            result = points_array
            if len(result) > 0:
                print(f"Standard XYZ mapping! Example point: [{result[0,0]:.2f}, {result[0,1]:.2f}, {result[0,2]:.2f}]")
        swap_time = time.time() - start_swap
        
        # Overall performance metrics
        total_time = time.time() - start_total
        print(f"Performance breakdown (ms): Total={total_time*1000:.1f}, "
              f"Downsample={downsample_time*1000:.1f}, Filter={filter_time*1000:.1f}, "
              f"Drake={drake_time*1000:.1f}, Swap={swap_time*1000:.1f}")
        print(f"Drake details (ms): Image={image_create_time*1000:.1f}, "
              f"Process={drake_process_time*1000:.1f}, Convert={conversion_time*1000:.1f}, "
              f"Filter={filter_points_time*1000:.1f}")
        
        return result
    
    def process_depth_image(self, depth_image, header):
        """
        Process depth image and publish point cloud
        :param depth_image: numpy array with depth values
        :param header: header from the original depth image message
        """
        try:
            start_time = time.time()
            
            # Convert depth image to point cloud using Drake
            points = self.drake_depth_to_pointcloud(depth_image)
            
            if points is not None and len(points) > 0:
                # Create PointCloud2 message
                cloud_msg = self.create_point_cloud2_msg(points, header=header)
                
                # Publish
                self.lc.publish("head_cam_pointcloud#sensor_msgs.PointCloud2", cloud_msg.encode())
                
                # Debug output
                processing_time = time.time() - start_time
                print(f"Published pointcloud with {len(points)} points in {processing_time*1000:.1f}ms")
                
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
    parser = argparse.ArgumentParser(description='Convert depth images to point clouds using Drake')
    parser.add_argument('--swap-y-z', action='store_true', help='Swap Y and Z axes for visualization')
    parser.add_argument('--downsample', type=int, default=1, help='Downsample factor (1=no downsampling, 2=half resolution, etc.)')
    parser.add_argument('--max-depth', type=float, default=10.0, help='Maximum depth to include in meters')
    parser.add_argument('--min-depth', type=float, default=0.1, help='Minimum depth to include in meters')
    args = parser.parse_args()
    
    # Initialize depth to pointcloud converter
    converter = DrakeDepthToPointcloudConverter(swap_y_z=args.swap_y_z)
    
    # Set additional parameters
    converter.downsample_factor = args.downsample
    converter.max_depth = args.max_depth
    converter.filter_threshold = args.min_depth
    
    try:
        print("Drake Depth to Pointcloud converter running with settings:")
        print(f"  Swap Y-Z axes: {args.swap_y_z}")
        print(f"  Downsample factor: {args.downsample}")
        print(f"  Depth range: {args.min_depth} to {args.max_depth} meters")
        print("Waiting for depth image and camera info messages...")
        
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

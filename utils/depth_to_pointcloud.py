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
    def __init__(self, swap_y_z=False):
        self.lc = lcm.LCM()
        self.lc_thread = None
        self.running = True
        
        # Initialize last received messages
        self.last_depth_image = None
        self.last_depth_stamp = 0
        self.camera_info = None
        self.camera_info_received = False
        
        # Cache for optimization
        self.cached_x_map = None
        self.cached_y_map = None
        self.cached_dimensions = None
        
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
            
            # Clear the cache when camera info changes
            self.cached_x_map = None
            self.cached_y_map = None
            self.cached_dimensions = None
            
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
    
    def create_point_cloud2_msg(self, points, rgb=None, header=None):
        """
        Create a PointCloud2 message from point data
        :param points: Nx3 numpy array of (x, y, z) points
        :param rgb: Nx3 numpy array of (r, g, b) colors (optional)
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
        
        # Create XYZRGB point cloud
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
    
    def generate_point_cloud(self, depth_image):
        """
        Convert a depth image to a point cloud (optimized version)
        :param depth_image: numpy array with depth values
        :return: Nx3 numpy array of (x, y, z) points
        """
        # Check if we need to create the lookup maps
        height, width = depth_image.shape
        
        if (self.cached_x_map is None or self.cached_y_map is None or 
            self.cached_dimensions != (height, width)):
            
            # Get camera matrix
            if self.camera_info is None:
                print("Camera info not available")
                return None
            
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
                fx = fx / self.downsample_factor
                fy = fy / self.downsample_factor
                cx = cx / self.downsample_factor
                cy = cy / self.downsample_factor
            
            # Create coordinate maps (these can be reused for all frames with the same dimensions)
            # Using meshgrid is more efficient than nested loops
            v, u = np.mgrid[:height, :width]
            
            # Pre-compute fixed parts of the projection
            self.cached_x_map = (u - cx) / fx
            self.cached_y_map = (v - cy) / fy
            self.cached_dimensions = (height, width)
        
        else:
            # Apply downsampling if necessary
            if self.downsample_factor > 1:
                depth_image = cv2.resize(depth_image, 
                                        (self.cached_dimensions[1], self.cached_dimensions[0]), 
                                        interpolation=cv2.INTER_NEAREST)
        
        # Apply filters to depth image
        valid_mask = np.logical_and(
            depth_image > self.filter_threshold,  # Remove points too close
            depth_image < self.max_depth         # Remove points too far
        )
        
        # Extract valid depth values and coordinates
        valid_depths = depth_image[valid_mask]
        valid_x_map = self.cached_x_map[valid_mask]
        valid_y_map = self.cached_y_map[valid_mask]
        
        # Compute 3D points
        # Using multiple small arrays is more cache-friendly than a single large array
        # This is faster than appending to a list or concatenating arrays
        points_x = valid_x_map * valid_depths
        points_y = valid_y_map * valid_depths
        points_z = valid_depths
        
        # Stack into a single array only at the end
        if self.swap_y_z:
            # Swap Y and Z axes for correct orientation in visualization
            # Standard camera convention: X right, Y down, Z forward
            # After swap: X right, Z down (up becomes positive), Y forward (depth)
            return np.column_stack((points_x, points_z, -points_y))  # Negate Y to flip axis
        else:
            # Use standard camera coordinate system 
            return np.column_stack((points_x, points_y, points_z))
    
    def process_depth_image(self, depth_image, header):
        """
        Process depth image and publish point cloud
        :param depth_image: numpy array with depth values
        :param header: header from the original depth image message
        """
        try:
            start_time = time.time()
            
            # Generate point cloud
            points = self.generate_point_cloud(depth_image)
            
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
    parser = argparse.ArgumentParser(description='Convert depth images to point clouds')
    parser.add_argument('--swap-y-z', action='store_true', help='Swap Y and Z axes for visualization')
    parser.add_argument('--downsample', type=int, default=1, help='Downsample factor (1=no downsampling, 2=half resolution, etc.)')
    parser.add_argument('--max-depth', type=float, default=10.0, help='Maximum depth to include in meters')
    parser.add_argument('--min-depth', type=float, default=0.1, help='Minimum depth to include in meters')
    args = parser.parse_args()
    
    # Initialize depth to pointcloud converter
    converter = DepthToPointcloudConverter(swap_y_z=args.swap_y_z)
    
    # Set additional parameters
    converter.downsample_factor = args.downsample
    converter.max_depth = args.max_depth
    converter.filter_threshold = args.min_depth
    
    try:
        print("Depth to pointcloud converter running with settings:")
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
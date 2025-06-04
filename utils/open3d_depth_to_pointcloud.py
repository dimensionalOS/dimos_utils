#!/usr/bin/env python3
from contextlib import contextmanager
import numpy as np
import lcm
import threading
import time
import cv2
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from lcm_msgs.sensor_msgs import Image, CameraInfo, PointCloud2, PointField, JointState
from lcm_msgs.std_msgs import Header

# Open3D imports
import open3d as o3d

# Imports for tf_lcm_py
import tf_lcm_py
import lcm_msgs
import signal
import sys
import atexit
from datetime import datetime
from pydrake.all import MinimumDistanceLowerBoundConstraint, MultibodyPlant, Parser, DiagramBuilder, AddMultibodyPlantSceneGraph, MeshcatVisualizer, StartMeshcat, RigidTransform, Role, RollPitchYaw, RotationMatrix, Solve, InverseKinematics, MeshcatVisualizerParams, MinimumDistanceLowerBoundConstraint, DoDifferentialInverseKinematics, DifferentialInverseKinematicsStatus, DifferentialInverseKinematicsParameters, DepthImageToPointCloud
import traceback
import os
from pydrake.multibody.tree import RevoluteJoint, PrismaticJoint


from typing import Optional
class Open3DDepthToPointcloudConverter:
    def __init__(self, swap_y_z=False):
        self.joint_states_dict = {}
        self.lc = lcm.LCM()
        self.lc_thread = None
        self._resources_to_cleanup = []

        # Register cleanup at exit
        atexit.register(self.cleanup_resources)

        # Initialize tf resources once and reuse them
        self.buffer = tf_lcm_py.Buffer(30.0)
        self._resources_to_cleanup.append(self.buffer)
        with self.safe_lcm_instance() as lcm_instance:
            self.tf_lcm_instance = lcm_instance
            self._resources_to_cleanup.append(self.tf_lcm_instance)
            # Create TransformListener with our LCM instance and buffer
            self.listener = tf_lcm_py.TransformListener(self.tf_lcm_instance, self.buffer)
            self._resources_to_cleanup.append(self.listener)

        self.running = True
        
        # Initialize last received messages
        self.last_depth_image = None
        self.last_depth_stamp = 0
        self.camera_info = None
        self.camera_info_received = False
        self.frame_id = "camera"
        self.cloud_frame_id = None
        
        # Open3D camera intrinsics
        self.o3d_intrinsic = None
        
        # Multithreading
        self.pool = ThreadPoolExecutor(max_workers=4)  # Adjust as needed
        
        # Optimization settings
        self.downsample_factor = 1  # Set to higher for downsampling (e.g., 2 for half resolution)
        self.filter_threshold = 0.1  # Filter out points closer than this value (in meters)
        self.max_depth = 10.0  # Filter out points further than this value (in meters)
        self.depth_scale = 1000.0  # Depth scale (1000.0 for mm to m conversion)
        self.depth_trunc = 10.0  # Truncate depth values beyond this distance
        
        # Coordinate system settings
        self.swap_y_z = swap_y_z  # If True, will swap Y and Z axes
        print(f"Point cloud axes: {'Y and Z axes swapped' if self.swap_y_z else 'Standard XYZ mapping'}")
        
        # Subscribe to topics
        self.lc.subscribe("head_cam_depth#sensor_msgs.Image", self.depth_callback)
        self.lc.subscribe("head_cam_depth_info#sensor_msgs.CameraInfo", self.camera_info_callback)
        self.lc.subscribe("joint_states#sensor_msgs.JointState", self._joint_states_callback)
        
        # Start LCM thread
        self.start_lcm_thread()
        
    def initialize_open3d_intrinsic(self):
        """Initialize the Open3D camera intrinsic with the current camera info"""
        if self.camera_info is None:
            return False
        
        # Extract camera parameters
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
        
        # Create Open3D camera intrinsic
        self.o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width, height=height,
            fx=fx, fy=fy, cx=cx, cy=cy
        )
        
        # print(f"Initialized Open3D camera intrinsic with parameters:")
        # print(f"  Size: {width}x{height}")
        # print(f"  Focal: ({fx:.1f}, {fy:.1f})")
        # print(f"  Center: ({cx:.1f}, {cy:.1f})")
        
        return True

    @contextmanager
    def safe_lcm_instance(self):
        """Context manager for safely managing LCM instance lifecycle"""
        lcm_instance = tf_lcm_py.LCM()
        try:
            yield lcm_instance
        finally:
            pass

    def cleanup_resources(self):
        """Clean up resources before exiting"""
        # Only clean up once when exiting
        print("Cleaning up resources...")
        # Force cleanup of resources in reverse order (last created first)
        for resource in reversed(self._resources_to_cleanup):
            try:
                # For objects like TransformListener that might have a close or shutdown method
                if hasattr(resource, 'close'):
                    resource.close()
                elif hasattr(resource, 'shutdown'):
                    resource.shutdown()
                
                # Explicitly delete the resource
                del resource
            except Exception as e:
                print(f"Error during cleanup: {e}")
        
        # Clear the resources list
        self._resources_to_cleanup = []

    def _joint_states_callback(self, channel, data):
        """
        Callback for joint states. Builds a dict: joint_name -> joint_position.
        """
        try:
            msg = JointState.decode(data)
            # msg.name is a list of joint‐names, msg.position is a list of floats
            self.joint_states_dict = {
                name: pos for name, pos in zip(msg.name, msg.position)
            }
            # For debugging:
            # print(f"Received joint states dict: {self.joint_states_dict}")
        except Exception as e:
            print(f"Error decoding joint states: {e}")
            traceback.print_exc()


    def get_transform(self,
                      source_frame: str,
                      target_frame: str,
                      urdf_path: Optional[str] = os.path.abspath("../assets/devkit_base_descr.urdf")
                      ) -> RigidTransform:
        """
        Build a small MultibodyPlant from the given URDF, set all joints
        according to whatever was most recently stored in self.joint_states_dict,
        and return the RigidTransform from source_frame --> target_frame.

        Assumes that self.joint_states_dict was populated by _joint_states_callback,
        which maps joint_name -> position.

        Args:
            source_frame:  name of the “from” frame (as defined in the URDF).
            target_frame:  name of the “to” frame (as defined in the URDF).
            urdf_path:     absolute (or relative) path to your URDF file.

        Returns:
            RigidTransform: pose of `target_frame` expressed in `source_frame` coordinates.
        """
        # List all joints in the correct kinematic order:
        kinematic_chain_joints = [
            "pillar_platform_joint",
            "pan_tilt_pan_joint",
            "pan_tilt_head_joint",   
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        # 1) Build a brand‐new MultibodyPlant + SceneGraph from the URDF:
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        parser = Parser(plant)
        parser.AddModelsFromUrl(f"file://{urdf_path}")
        plant.Finalize()

        # 2) Create a fresh Context:
        context = plant.CreateDefaultContext()

        # 3) For each joint in our 7-DOF chain, check if we've got a value in self.joint_states_dict.
        #    If so, set it; otherwise leave at default (zero).
        for joint_name in kinematic_chain_joints:
            if joint_name in self.joint_states_dict:
                pos = self.joint_states_dict[joint_name]
                joint = plant.GetJointByName(joint_name)

                # If it's a revolute joint, use set_angle(...)
                if isinstance(joint, RevoluteJoint):
                    joint.set_angle(context, pos)

                # If it's a prismatic joint, use set_translation(...)
                elif isinstance(joint, PrismaticJoint):
                    joint.set_translation(context, pos)

                # Otherwise (e.g. multi-DOF), fall back to set_positions([...]) if available:
                else:
                    # joint.num_positions() should tell you how many DOFs this joint has.
                    joint.set_positions(context, np.array([pos] * joint.num_positions()))

            # If joint_name not in dictionary, we leave it at its default (zero)
            else:
                pass

        # 4) Look up the two Frame objects by name:
        frame_A = plant.GetFrameByName(source_frame)
        frame_B = plant.GetFrameByName(target_frame)

        # 5) Compute the relative transform X_A_B = pose_of(B) in A’s coords:
        X_A_B = plant.CalcRelativeTransform(context, frame_A, frame_B)

        return X_A_B

        
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
            
            # TODO: Figure out a way to toggle this based on whether or not this 
            # script is being run standalone or called from another script.
            
            # If we have camera info, convert depth to point cloud in a separate thread
            # if self.camera_info_received:
            #     self.pool.submit(self.process_depth_image, depth_img, img_msg.header)
                
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
            
            # Initialize Open3D intrinsic with this camera info
            success = self.initialize_open3d_intrinsic()
            
            # if success:
            #     print(f"Received camera info: f={self.camera_info.K[0]:.1f}, width={self.camera_info.width}, height={self.camera_info.height}")
            
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
        
        if self.cloud_frame_id is None:
            cloud_msg.header.frame_id = self.frame_id
        else:
            cloud_msg.header.frame_id = self.cloud_frame_id
        
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
    
    def open3d_depth_to_pointcloud(self, depth_image):
        """
        Convert a depth image to a point cloud using the pinhole camera model
        :param depth_image: numpy array with depth values (float32, in meters)
        :return: Nx3 numpy array of (x, y, z) points
        """
        if self.o3d_intrinsic is None:
            print("Camera intrinsic not initialized")
            return None
        
        # Detailed timing for performance analysis
        start_total = time.time()
        
        # Get camera parameters
        width = self.o3d_intrinsic.width
        height = self.o3d_intrinsic.height
        fx = self.o3d_intrinsic.get_focal_length()[0]  # Focal length x
        fy = self.o3d_intrinsic.get_focal_length()[1]  # Focal length y
        cx = self.o3d_intrinsic.get_principal_point()[0]  # Principal point x
        cy = self.o3d_intrinsic.get_principal_point()[1]  # Principal point y
        
        # Apply downsampling if requested (this can significantly improve performance)
        start_downsample = time.time()
        if self.downsample_factor > 1:
            depth_image = cv2.resize(depth_image, 
                                    (width, height), 
                                    interpolation=cv2.INTER_NEAREST)
        downsample_time = time.time() - start_downsample
        
        # Apply filters to depth image before conversion
        start_filter = time.time()
        # Only create a copy if filters are applied
        if self.filter_threshold > 0 or self.max_depth < float('inf'):
            depth_mask = (depth_image >= self.filter_threshold) & (depth_image <= self.max_depth)
        else:
            depth_mask = np.ones_like(depth_image, dtype=bool)
        filter_time = time.time() - start_filter
        
        # Start the conversion process
        start_conversion = time.time()
        
        # Create meshgrid for pixel coordinates
        v, u = np.mgrid[0:height, 0:width]
        
        # Apply the pinhole camera model
        # z = depth value
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        z = depth_image
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Stack coordinates into a single array
        points = np.stack((x, y, z), axis=2)
        
        # Apply depth mask to filter out invalid points
        valid_points = points[depth_mask]
        
        conversion_time = time.time() - start_conversion
        
        # Create Open3D point cloud for potential operations
        start_o3d = time.time()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(valid_points)
        o3d_time = time.time() - start_o3d
        
        # Apply coordinate transformation if needed
        start_swap = time.time()
        result = np.asarray(pcd.points)
        if self.swap_y_z:
            # Swap Y and Z axes for visualization
            # Standard camera convention: X right, Y down, Z forward
            # After swap: X right, Z down (up becomes positive), Y forward (depth)
            result = np.column_stack((result[:, 0], result[:, 2], -result[:, 1]))
            if len(result) > 0:
                print(f"Y-Z axes swapped! Example point: [{result[0,0]:.2f}, {result[0,1]:.2f}, {result[0,2]:.2f}]")
        else:
            if len(result) > 0:
                print(f"Standard XYZ mapping! Example point: [{result[0,0]:.2f}, {result[0,1]:.2f}, {result[0,2]:.2f}]")
        swap_time = time.time() - start_swap
        
        # Overall performance metrics
        total_time = time.time() - start_total
        print(f"Performance breakdown (ms): Total={total_time*1000:.1f}, "
              f"Downsample={downsample_time*1000:.1f}, Filter={filter_time*1000:.1f}, "
              f"Conversion={conversion_time*1000:.1f}, Open3D={o3d_time*1000:.1f}, Swap={swap_time*1000:.1f}")
        print(f"Generated point cloud with {len(result)} points")
        
        return result
    
    def transform_point_cloud_with_open3d(self,
                                          points_np: np.ndarray,
                                          tf: RigidTransform) -> np.ndarray:
        """
        Transforms a point cloud using Open3D given a Drake RigidTransform.

        Args:
            points_np (np.ndarray): Nx3 array of 3D points.
            tf (RigidTransform): Drake transform to apply.

        Returns:
            np.ndarray: Nx3 array of transformed 3D points.
        """
        if points_np.ndim != 2 or points_np.shape[1] != 3:
            print("Input point cloud must have shape Nx3.")
            return points_np

        # 1) Convert Drake RigidTransform --> 4×4 NumPy matrix
        tf_matrix = np.eye(4)

        # rotation() is a RotationMatrix.  To get the 3×3 matrix:
        R_mat = tf.rotation().matrix()      # type: ignore[attr-defined]
        t_vec = tf.translation()            # returns a length-3 NumPy array

        tf_matrix[:3, :3] = R_mat
        tf_matrix[:3, 3] = t_vec

        # (Optional) If you want to log the quaternion for debugging:
        # q = tf.rotation().ToQuaternion()   # This is a pydrake.common.eigen_geometry.Quaternion
        # qw, qx, qy, qz = q.w(), q.x(), q.y(), q.z()
        #
        # print(f"Quaternion = (w={qw:.6f}, x={qx:.6f}, y={qy:.6f}, z={qz:.6f})")
        # print(f"Translation = ({t_vec[0]:.6f}, {t_vec[1]:.6f}, {t_vec[2]:.6f})")

        # 2) Create an Open3D point cloud from points_np
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))

        # 3) Apply the 4×4 transform matrix
        pcd.transform(tf_matrix)

        # 4) Return the transformed points as an Nx3 NumPy array
        return np.asarray(pcd.points)

    
    def process_depth_image(self, depth_image, header):
        """
        Process depth image and publish point cloud
        :param depth_image: numpy array with depth values
        :param header: header from the original depth image message
        """
        try:
            start_time = time.time()

            # Set target frame for cloud
            self.cloud_frame_id = "world"
            
            # Convert depth image to point cloud using Open3D
            initial_points = self.open3d_depth_to_pointcloud(depth_image)
            
            if initial_points is None or len(initial_points) == 0:
                print("No points generated from depth image")
                return
                
            # Get transform from camera frame to world frame
            transform = self.get_transform(self.cloud_frame_id, self.frame_id)
            # transform = None
            
            if transform is None:
                # print("No transform found, using untransformed point cloud")
                points = initial_points
            else:
                # Transform points to world frame
                points = self.transform_point_cloud_with_open3d(initial_points, transform)
            
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
    parser = argparse.ArgumentParser(description='Convert depth images to point clouds using Open3D')
    parser.add_argument('--swap-y-z', type=bool, default=True, help='Swap Y and Z axes for visualization')
    parser.add_argument('--downsample', type=int, default=1, help='Downsample factor (1=no downsampling, 2=half resolution, etc.)')
    parser.add_argument('--max-depth', type=float, default=10.0, help='Maximum depth to include in meters')
    parser.add_argument('--min-depth', type=float, default=0.1, help='Minimum depth to include in meters')
    args = parser.parse_args()
    
    # Initialize depth to pointcloud converter
    converter = Open3DDepthToPointcloudConverter(swap_y_z=args.swap_y_z)
    
    # Configure optimization settings
    converter.downsample_factor = args.downsample
    converter.filter_threshold = args.min_depth
    converter.max_depth = args.max_depth
    
    print(f"Open3D Depth to Pointcloud Converter")
    print(f"Settings:")
    print(f"  - Swap Y-Z axes: {args.swap_y_z}")
    print(f"  - Downsample factor: {args.downsample}")
    print(f"  - Depth range: {args.min_depth}m to {args.max_depth}m")
    
    try:
        # Keep the main thread alive
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up
        converter.stop()
        print("Converter stopped.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import numpy as np
import argparse
import time
import os
import lcm
import zmq
import json
from lcm_msgs.sensor_msgs import JointState
from pydrake.all import (
    MultibodyPlant,
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    StartMeshcat,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    Simulator,
    DoDifferentialInverseKinematics,
    DifferentialInverseKinematicsStatus,
    DifferentialInverseKinematicsParameters,
    InverseKinematics,
    Solve,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)


class XArmOpenFTController:
    def __init__(self, use_ik_control=False, enable_real_robot=False, enable_sensor_collection=False, xarm_ip=None):
        # Start meshcat first
        self.meshcat = StartMeshcat()
        
        # Store control mode
        self.use_ik_control = use_ik_control
        self.enable_real_robot = enable_real_robot
        self.enable_sensor_collection = enable_sensor_collection
        self.xarm_ip = xarm_ip
        
        # Get initial positions from xARM if IP provided
        self.xarm_initial_positions = None
        if self.xarm_ip:
            print(f"\n{'='*60}")
            print(f"Attempting to connect to xARM at {self.xarm_ip}")
            print(f"{'='*60}")
            self.xarm_initial_positions = self.get_xarm_positions()
            if self.xarm_initial_positions is None:
                print("WARNING: Failed to get xARM positions, using default positions")
                print("Check that:")
                print("  1. xARM is powered on and connected")
                print("  2. IP address is correct") 
                print("  3. xarm-python-sdk is installed: pip install xarm-python-sdk")
                print(f"{'='*60}\n")
        
        # Initialize LCM if real robot mode is enabled
        if self.enable_real_robot:
            self.lc = lcm.LCM()
            self.joint_state_msg = JointState()
            print("LCM initialized for real robot control")
            print("Joint states will be published when positions change")
        
        # Initialize ZMQ for sensor data collection if enabled
        self.sensor_data = None
        if self.enable_sensor_collection:
            try:
                self.zmq_context = zmq.Context()
                self.sensor_socket = self.zmq_context.socket(zmq.SUB)
                self.sensor_socket.connect("tcp://localhost:5555")
                self.sensor_socket.setsockopt_string(zmq.SUBSCRIBE, "")
                self.sensor_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
                print("ZMQ initialized for sensor data collection on port 5555")
            except Exception as e:
                print(f"Warning: Failed to initialize ZMQ for sensor collection: {e}")
                self.enable_sensor_collection = False
        
        # Setup the simulation
        self.setup_simulation()
        
        # Initial publish to show robot
        self.diagram.ForcedPublish(self.diagram_context)
        
        # Setup controls based on mode
        if self.use_ik_control:
            self.setup_ik_controls()
        else:
            self.setup_joint_controls()
        
        # Create visualizations for both modes
        self.create_gripper_com_visualization()
        self.create_openft_frame_visualization()
        self.create_world_frame_visualization()
        self.create_force_vector_visualization()
            
        # Publish again after setting up controls
        self.diagram.ForcedPublish(self.diagram_context)
        
        print(f"\n{'='*60}")
        print(f"Meshcat URL: {self.meshcat.web_url()}")
        print(f"Open this URL in your browser to view the robot")
        print(f"{'='*60}\n")
    
    def get_xarm_positions(self):
        """Get current joint positions from xARM robot."""
        try:
            from xarm.wrapper import XArmAPI
            
            print(f"Connecting to xARM at {self.xarm_ip}...")
            arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            
            # Clear any errors and enable
            arm.clean_error()
            arm.clean_warn()
            
            # Get current joint angles (returns [code, angles])
            code, angles = arm.get_servo_angle(is_radian=True)
            
            if code == 0 and angles:
                print(f"Got xARM joint positions (radians):")
                for i, angle in enumerate(angles[:6]):
                    print(f"  joint{i+1}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")
                
                # Also get and print TCP pose for verification
                code, tcp_pose = arm.get_position(is_radian=True)
                if code == 0 and tcp_pose:
                    print(f"xARM TCP pose (from robot):")
                    # Note: xARM returns position in mm, need to convert to meters for comparison
                    print(f"  Position: x={tcp_pose[0]:.1f} mm ({tcp_pose[0]/1000:.3f} m)")
                    print(f"           y={tcp_pose[1]:.1f} mm ({tcp_pose[1]/1000:.3f} m)")
                    print(f"           z={tcp_pose[2]:.1f} mm ({tcp_pose[2]/1000:.3f} m)")
                    print(f"  Rotation: rx={tcp_pose[3]:.3f}, ry={tcp_pose[4]:.3f}, rz={tcp_pose[5]:.3f} rad")
                
                # Double-check by also getting angles in degrees
                code_deg, angles_deg = arm.get_servo_angle(is_radian=False)
                if code_deg == 0 and angles_deg:
                    print(f"\nDouble-check - xARM joint positions (degrees):")
                    for i, angle in enumerate(angles_deg[:6]):
                        print(f"  joint{i+1}: {angle:.2f} deg (converted: {np.radians(angle):.4f} rad)")
                
                # xARM returns 6 values for xARM6
                if len(angles) >= 6:
                    arm.disconnect()
                    return angles[:6]  # Return first 6 joint angles
                else:
                    print(f"Warning: Expected 6 joint angles, got {len(angles)}")
            else:
                print(f"Failed to get xARM positions, code: {code}")
            
            arm.disconnect()
            
        except ImportError:
            print("Error: xarm library not installed. Install with: pip install xarm-python-sdk")
        except Exception as e:
            print(f"Error connecting to xARM: {e}")
            print("Make sure the xARM is powered on and the IP address is correct")
        
        return None
    
    def setup_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()
        
        # Create diagram builder
        self.builder = DiagramBuilder()
        
        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.001  # Discrete time for mimic joints
        )
        
        # Parse URDF
        parser = Parser(self.plant)
        package_path = os.path.dirname(os.path.abspath(__file__))
        
        # Add package path for mesh loading
        parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))
        
        # Load the URDF (using the version with .obj files)
        urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
        self.model_instances = parser.AddModels(urdf_path)
        self.model_instance = self.model_instances[0] if self.model_instances else None
        
        # The URDF already has a world_joint connecting to world, so no need to weld
        
        # Get end-effector frame
        try:
            self.end_effector_frame = self.plant.GetFrameByName("link_tcp")
            self.end_effector_body = self.plant.GetBodyByName("link_tcp")
            print("Using link_tcp as end-effector frame")
        except:
            try:
                self.end_effector_frame = self.plant.GetFrameByName("link6")
                self.end_effector_body = self.plant.GetBodyByName("link6")
                print("Using link6 as end-effector frame")
            except:
                print("Warning: Could not find end-effector frame")
                self.end_effector_frame = None
                self.end_effector_body = None
        
        # Get gripper COM frame for visualization
        try:
            self.gripper_com_body = self.plant.GetBodyByName("xarm_gripper_com")
            print("Found xarm_gripper_com link for visualization")
        except:
            print("Warning: Could not find xarm_gripper_com link")
            self.gripper_com_body = None
        
        # Get link_openft for force/torque calculations
        try:
            self.openft_body = self.plant.GetBodyByName("link_openft")
            print("Found link_openft for force/torque calculations")
        except:
            print("Warning: Could not find link_openft")
            self.openft_body = None
        
        # Gripper parameters
        self.gripper_mass = 0.806  # kg
        self.gravity = np.array([0, 0, -9.81])  # m/s^2 in world frame
        
        # Finalize the plant
        self.plant.Finalize()
        
        # Add visualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat
        )
        
        # Build the diagram
        self.diagram = self.builder.Build()
        
        # Create contexts
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)
        
        # Get joint names
        self.arm_joint_names = [f"joint{i+1}" for i in range(6)]
        self.gripper_joint_name = "drive_joint"
        
        # Set initial positions
        initial_positions = np.zeros(self.plant.num_positions())
        
        # Use xARM positions if available, otherwise use default configuration
        if self.xarm_initial_positions is not None:
            print("\nInitializing Drake with xARM joint positions:")
            initial_joint_angles = self.xarm_initial_positions
        else:
            # Default configuration to avoid singularities/gimbal lock
            initial_joint_angles = [0.0, -0.5, -0.5, 0.0, -0.5, 0.0]  # Slightly bent elbow configuration
        
        # Set joint positions and print for verification
        for i, joint_name in enumerate(self.arm_joint_names):
            try:
                joint = self.plant.GetJointByName(joint_name)
                joint_index = joint.position_start()
                if i < len(initial_joint_angles):
                    initial_positions[joint_index] = initial_joint_angles[i]
                    if self.xarm_initial_positions is not None:
                        print(f"  Setting {joint_name} (index {joint_index}) = {initial_joint_angles[i]:.4f} rad")
            except Exception as e:
                print(f"  Error setting {joint_name}: {e}")
        
        self.plant.SetPositions(self.plant_context, initial_positions)
        
        # Verify positions were set correctly
        if self.xarm_initial_positions is not None:
            actual_positions = self.plant.GetPositions(self.plant_context)
            print("\nVerifying Drake positions after setting:")
            for i, joint_name in enumerate(self.arm_joint_names):
                joint = self.plant.GetJointByName(joint_name)
                joint_index = joint.position_start()
                print(f"  {joint_name}: {actual_positions[joint_index]:.4f} rad")
        
        # Reset camera for better view
        self.meshcat.SetCameraPose(
            camera_in_world=[2.0, 2.0, 1.5],
            target_in_world=[0.0, 0.0, 0.3]
        )
    
    def setup_joint_controls(self):
        """Setup sliders for direct joint control."""
        print("Setting up joint control sliders...")
        
        # For joint control mode, if we don't have xARM positions, set to zero
        if self.xarm_initial_positions is None:
            zero_positions = np.zeros(self.plant.num_positions())
            self.plant.SetPositions(self.plant_context, zero_positions)
        
        # Create joint sliders
        for i, joint_name in enumerate(self.arm_joint_names):
            joint = self.plant.GetJointByName(joint_name)
            lower = joint.position_lower_limit()
            upper = joint.position_upper_limit()
            # Handle single DOF joints
            if hasattr(lower, '__getitem__'):
                lower = lower[0]
                upper = upper[0]
            
            # Get initial value - use xARM position if available, otherwise current position
            if self.xarm_initial_positions is not None and i < len(self.xarm_initial_positions):
                initial_value = self.xarm_initial_positions[i]
            else:
                # Get current position from plant context
                current_positions = self.plant.GetPositions(self.plant_context)
                initial_value = current_positions[joint.position_start()]
            
            # Add slider with initial value
            self.meshcat.AddSlider(
                name=joint_name,
                min=lower,
                max=upper,
                step=0.01,
                value=initial_value
            )
            print(f"  Added slider for {joint_name}: [{lower:.2f}, {upper:.2f}] = {initial_value:.3f}")
        
        # Gripper slider
        gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
        g_lower = gripper_joint.position_lower_limit()
        g_upper = gripper_joint.position_upper_limit()
        if hasattr(g_lower, '__getitem__'):
            g_lower = g_lower[0]
            g_upper = g_upper[0]
        
        self.meshcat.AddSlider(
            name=self.gripper_joint_name,
            min=g_lower,
            max=g_upper,
            step=0.01,
            value=0.0
        )
        print(f"  Added slider for {self.gripper_joint_name}: [{g_lower:.2f}, {g_upper:.2f}]")
    
    def setup_ik_controls(self):
        """Setup sliders for inverse kinematics control."""
        print("Setting up IK control sliders...")
        
        # Get current end-effector pose from the plant (which has xARM positions if available)
        if self.end_effector_body:
            ee_pose = self.plant.EvalBodyPoseInWorld(
                self.plant_context, self.end_effector_body
            )
            current_pos = ee_pose.translation()
            rpy = RollPitchYaw(ee_pose.rotation())
            
            # If we got xARM positions, print the computed end-effector pose
            if self.xarm_initial_positions is not None:
                print(f"  Computed end-effector pose from xARM positions (Drake FK):")
                print(f"    Position: x={current_pos[0]:.3f} m, y={current_pos[1]:.3f} m, z={current_pos[2]:.3f} m")
                print(f"    Orientation (RPY): r={rpy.roll_angle():.3f}, p={rpy.pitch_angle():.3f}, y={rpy.yaw_angle():.3f} rad")
                print(f"  Compare this with the xARM TCP pose printed above to verify forward kinematics match")
        else:
            current_pos = np.array([0.3, 0.0, 0.3])
            rpy = RollPitchYaw(0, 0, 0)
        
        # Position sliders - initialize to current end-effector position
        self.meshcat.AddSlider("target_x", -0.8, 0.8, 0.01, current_pos[0])
        self.meshcat.AddSlider("target_y", -0.8, 0.8, 0.01, current_pos[1])
        self.meshcat.AddSlider("target_z", 0.0, 1.0, 0.01, current_pos[2])
        print(f"  Added position sliders (x,y,z) initialized to current pose")
        
        # Orientation sliders (in radians) - initialize to current end-effector orientation
        self.meshcat.AddSlider("target_roll", -np.pi, np.pi, 0.01, rpy.roll_angle())
        self.meshcat.AddSlider("target_pitch", -np.pi, np.pi, 0.01, rpy.pitch_angle())
        self.meshcat.AddSlider("target_yaw", -np.pi, np.pi, 0.01, rpy.yaw_angle())
        print(f"  Added orientation sliders (roll,pitch,yaw) initialized to current pose")
        
        # Gripper slider
        self.meshcat.AddSlider("gripper", 0.0, 0.85, 0.01, 0.0)
        print(f"  Added gripper slider")
        
        # Setup differential IK parameters
        self.diff_ik_params = DifferentialInverseKinematicsParameters(
            self.plant.num_positions(),
            self.plant.num_velocities()
        )
        
        # Set timestep
        self.dt = 0.05
        self.diff_ik_params.set_time_step(self.dt)
        
        # Set joint limits
        q_lower = self.plant.GetPositionLowerLimits()
        q_upper = self.plant.GetPositionUpperLimits()
        self.diff_ik_params.set_joint_position_limits((q_lower, q_upper))
        
        # Set velocity limits
        v_lower = self.plant.GetVelocityLowerLimits()
        v_upper = self.plant.GetVelocityUpperLimits()
        self.diff_ik_params.set_joint_velocity_limits((v_lower, v_upper))
        
        # Note: set_end_effector_velocity_gain may not be available in all Drake versions
        # If you have a newer version, you can uncomment the following:
        # self.diff_ik_params.set_end_effector_velocity_gain(2.0)
        
        # Enable all 6 DOF for end-effector control (3 translation + 3 rotation)
        # This ensures all rotational DOF are properly controlled
        velocity_flag = np.ones(6, dtype=bool)  # [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
        self.diff_ik_params.set_end_effector_velocity_flag(velocity_flag)
        
        # Create target visualization
        self.create_target_visualization()
        
        # Position the target frame at the current end-effector pose
        if self.end_effector_body:
            ee_pose = self.plant.EvalBodyPoseInWorld(
                self.plant_context, self.end_effector_body
            )
            self.meshcat.SetTransform("target_frame", ee_pose)
            print(f"  Target frame initialized at current end-effector pose")
    
    def create_target_visualization(self):
        """Create visualization for the target pose."""
        axis_length = 0.15
        axis_radius = 0.005
        
        # X-axis (red)
        self.meshcat.SetObject(
            "target_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1, 0, 0, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green)
        self.meshcat.SetObject(
            "target_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0, 1, 0, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue)
        self.meshcat.SetObject(
            "target_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0, 0, 1, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Origin cube
        self.meshcat.SetObject(
            "target_frame/origin",
            Box([0.02, 0.02, 0.02]),
            Rgba(0.5, 0.5, 0.5, 0.5)
        )
    
    def create_gripper_com_visualization(self):
        """Create visualization axes for the xarm_gripper_com link."""
        if not self.gripper_com_body:
            return
            
        axis_length = 0.1
        axis_radius = 0.003
        
        # X-axis (red) - slightly darker/different shade to distinguish from target
        self.meshcat.SetObject(
            "gripper_com_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(0.8, 0.2, 0.2, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - slightly darker/different shade
        self.meshcat.SetObject(
            "gripper_com_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.2, 0.8, 0.2, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - slightly darker/different shade
        self.meshcat.SetObject(
            "gripper_com_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.2, 0.2, 0.8, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Small sphere at origin to mark the COM position
        self.meshcat.SetObject(
            "gripper_com_frame/origin",
            Sphere(0.01),
            Rgba(1.0, 1.0, 0.0, 0.8)  # Yellow sphere
        )
        
        print("Created gripper COM axes visualization (darker colors)")
    
    def create_openft_frame_visualization(self):
        """Create visualization axes for the link_openft frame."""
        if not self.openft_body:
            return
            
        axis_length = 0.12
        axis_radius = 0.004
        
        # X-axis (red) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1.0, 0.3, 0.3, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.3, 1.0, 0.3, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.3, 0.3, 1.0, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Small cube at origin to mark the OpenFT sensor location
        self.meshcat.SetObject(
            "openft_frame/origin",
            Box([0.015, 0.015, 0.015]),
            Rgba(0.7, 0.7, 0.7, 0.8)  # Gray cube
        )
        
        print("Created OpenFT frame axes visualization (lighter colors)")
    
    def create_world_frame_visualization(self):
        """Create visualization axes for the world frame at origin."""
        axis_length = 0.3
        axis_radius = 0.005
        
        # X-axis (red) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1, 0, 0, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0, 1, 0, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0, 0, 1, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Add labels
        print("Created world frame axes visualization at origin")
    
    def create_force_vector_visualization(self):
        """Create visualization for the gravity force vector."""
        # Force vector (pointing down from COM)
        self.force_scale = 0.025  # Scale factor: 0.025 m per Newton (so ~0.2m for 8N)
        force_length = abs(self.gripper_mass * self.gravity[2]) * self.force_scale  # Length proportional to force
        
        # Create arrow shaft (cylinder)
        self.meshcat.SetObject(
            "force_vector/arrow",
            Cylinder(0.004, force_length),  # radius, length
            Rgba(1.0, 0.5, 0.0, 0.8)  # Orange color for force
        )
        
        # Create arrow head (cone using a tapered box or sphere)
        self.meshcat.SetObject(
            "force_vector/head",
            Sphere(0.01),  # Small sphere for arrow tip
            Rgba(1.0, 0.5, 0.0, 0.9)  # Slightly more opaque orange
        )
        
        print(f"Created force vector visualization (scaled: {self.force_scale} m/N)")
    
    def publish_joint_states(self):
        """Publish joint states to LCM for real robot control."""
        if not self.enable_real_robot:
            return
        
        # Get current joint positions
        q = self.plant.GetPositions(self.plant_context)
        
        # Prepare JointState message
        self.joint_state_msg.header.stamp.sec = int(time.time())
        self.joint_state_msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        self.joint_state_msg.header.frame_id = "base_link"
        
        # Set joint names and positions for joints 1-6 plus drive_joint
        joint_names = self.arm_joint_names.copy()  # ["joint1", "joint2", ..., "joint6"]
        joint_names.append("drive_joint")  # Add gripper drive joint
        
        self.joint_state_msg.name = joint_names
        self.joint_state_msg.position = []
        
        # Get positions for each arm joint
        for joint_name in self.arm_joint_names:
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            self.joint_state_msg.position.append(q[joint_idx])
        
        # Add drive_joint with fixed value of 0.85 (fully open gripper)
        self.joint_state_msg.position.append(0.85)
        
        # Set message lengths
        self.joint_state_msg.name_length = len(self.joint_state_msg.name)
        self.joint_state_msg.position_length = len(self.joint_state_msg.position)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []
        self.joint_state_msg.velocity_length = 0
        self.joint_state_msg.effort_length = 0
        
        # Publish to LCM
        try:
            self.lc.publish("joint_states#sensor_msgs.JointState", self.joint_state_msg.encode())
        except Exception as e:
            print(f"Error publishing joint state: {e}")
    
    def get_latest_sensor_data(self):
        """Get the latest sensor data from ZMQ."""
        if not self.enable_sensor_collection:
            return None
        
        latest_data = None
        try:
            # Drain the socket to get the latest message
            while True:
                try:
                    data_str = self.sensor_socket.recv_string(zmq.NOBLOCK)
                    latest_data = json.loads(data_str)
                except zmq.Again:
                    # No more messages
                    break
                except json.JSONDecodeError as e:
                    print(f"Error decoding sensor JSON: {e}")
                    
            # Store the latest data
            if latest_data and 'sensor_moving_averages' in latest_data:
                self.sensor_data = latest_data
                
        except Exception as e:
            print(f"Error getting sensor data: {e}")
            
        return self.sensor_data
    
    def update_force_vector(self):
        """Update the position and orientation of the force vector."""
        if not self.gripper_com_body:
            return
            
        # Get COM position
        com_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.gripper_com_body
        )
        com_pos = com_pose_world.translation()
        
        # Calculate force vector length based on actual force magnitude
        force_magnitude = abs(self.gripper_mass * self.gravity[2])  # Should be ~7.9 N
        force_length = force_magnitude * self.force_scale
        
        # Force vector points straight down (gravity)
        # The cylinder's default orientation is along the Z-axis
        # We need to position it to start at COM and extend downward
        
        # Position the cylinder's center halfway down from COM
        arrow_center = com_pos + np.array([0, 0, -force_length/2])
        
        # No rotation needed - cylinder default orientation is along Z
        # Just translate it to the right position
        force_transform = RigidTransform(
            RotationMatrix(),  # Identity rotation (cylinder already points along Z)
            arrow_center
        )
        self.meshcat.SetTransform("force_vector/arrow", force_transform)
        
        # Position arrow head at the tip (end of force vector)
        head_position = com_pos + np.array([0, 0, -force_length])
        head_transform = RigidTransform(
            RotationMatrix(),
            head_position
        )
        self.meshcat.SetTransform("force_vector/head", head_transform)
    
    def calculate_openft_wrench(self):
        """Calculate forces and torques on link_openft due to gripper weight at COM.
        
        Note: The force in world frame is always [0, 0, -mg] because gravity 
        always points straight down regardless of robot orientation. The torque
        changes based on the lever arm from link_openft to the COM.
        """
        if not self.gripper_com_body or not self.openft_body:
            return None, None
        
        # Get poses in world frame
        com_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.gripper_com_body
        )
        openft_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        
        # Force due to gravity (in world frame)
        # This is ALWAYS [0, 0, -mg] in world frame because gravity points down
        force_world = self.gripper_mass * self.gravity  # F = mg = [0, 0, -7.907] N
        
        # Position of COM relative to openft origin (in world frame)
        r_com_to_openft = com_pose_world.translation() - openft_pose_world.translation()
        
        # Torque = r × F (cross product of position vector and force)
        # This changes based on robot configuration
        torque_world = np.cross(r_com_to_openft, force_world)
        
        return force_world, torque_world
    
    def run_joint_control(self):
        """Run the joint control mode."""
        print("\n" + "="*60)
        print("Joint Control Mode Active")
        print("Use the sliders in Meshcat to control individual joints")
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        iteration_count = 0
        print_interval = 100  # Print every 100 iterations (about 1 second at 10ms delay)
        
        # Track previous positions to detect changes
        previous_positions = None
        position_tolerance = 1e-6  # Tolerance for position change detection
        
        # Publish initial state if real robot mode
        if self.enable_real_robot:
            self.publish_joint_states()
            print("[Joint Control] Published initial joint states to LCM")
        
        try:
            while True:
                iteration_count += 1
                # Store old positions for comparison
                old_positions = self.plant.GetPositions(self.plant_context).copy()
                
                # Get current positions (all DOF)
                current_positions = self.plant.GetPositions(self.plant_context)
                
                # Update arm joint positions from sliders
                positions_changed = False
                for joint_name in self.arm_joint_names:
                    try:
                        joint = self.plant.GetJointByName(joint_name)
                        joint_index = joint.position_start()
                        value = self.meshcat.GetSliderValue(joint_name)
                        if abs(current_positions[joint_index] - value) > position_tolerance:
                            positions_changed = True
                        current_positions[joint_index] = value
                    except:
                        pass
                
                # Update gripper position
                try:
                    gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                    gripper_index = gripper_joint.position_start()
                    gripper_value = self.meshcat.GetSliderValue(self.gripper_joint_name)
                    if abs(current_positions[gripper_index] - gripper_value) > position_tolerance:
                        positions_changed = True
                    current_positions[gripper_index] = gripper_value
                except:
                    pass
                
                # Set all positions
                self.plant.SetPositions(self.plant_context, current_positions)
                
                # Publish joint states to LCM only if positions changed
                if positions_changed and self.enable_real_robot:
                    self.publish_joint_states()
                    # Only print first few times and then occasionally
                    if iteration_count < 5 or iteration_count % 100 == 0:
                        print(f"[Joint Control] Published joint states to LCM")
                
                # Update gripper COM axes visualization
                if self.gripper_com_body:
                    gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.gripper_com_body
                    )
                    self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                    
                    # Update force vector visualization
                    self.update_force_vector()
                
                # Update OpenFT frame axes visualization
                if self.openft_body:
                    openft_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.openft_body
                    )
                    self.meshcat.SetTransform("openft_frame", openft_pose)
                
                # Calculate and print forces/torques periodically
                if iteration_count % print_interval == 0:
                    force_world, torque_world = self.calculate_openft_wrench()
                    if force_world is not None and self.openft_body:
                        print(f"\n--- OpenFT Forces & Torques ---")
                        print(f"WORLD FRAME:")
                        print(f"  Force:  [{force_world[0]:7.3f}, {force_world[1]:7.3f}, {force_world[2]:7.3f}] N")
                        print(f"  Torque: [{torque_world[0]:7.4f}, {torque_world[1]:7.4f}, {torque_world[2]:7.4f}] N⋅m")
                        
                        # Transform to OpenFT local frame
                        openft_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.openft_body)
                        R_world_to_openft = openft_pose.rotation().transpose()
                        force_openft = R_world_to_openft @ force_world
                        torque_openft = R_world_to_openft @ torque_world
                        
                        print(f"OPENFT LOCAL FRAME:")
                        print(f"  Force:  [{force_openft[0]:7.3f}, {force_openft[1]:7.3f}, {force_openft[2]:7.3f}] N")
                        print(f"  Torque: [{torque_openft[0]:7.4f}, {torque_openft[1]:7.4f}, {torque_openft[2]:7.4f}] N⋅m")
                        print(f"  Force magnitude:  {np.linalg.norm(force_openft):.3f} N")
                        print(f"  Torque magnitude: {np.linalg.norm(torque_openft):.4f} N⋅m")
                
                # Publish visualization
                self.diagram.ForcedPublish(self.diagram_context)
                
                # Small delay
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
    
    def run_ik_control(self):
        """Run the inverse kinematics control mode."""
        print("\n" + "="*60)
        print("Inverse Kinematics Control Mode Active")
        print("Use the target pose sliders in Meshcat to control the end-effector")
        print("  - target_x/y/z: End-effector position (meters)")
        print("  - target_roll/pitch/yaw: End-effector orientation (radians)")
        print("  - gripper: Gripper opening (0=closed, 0.85=open)")
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        if not self.end_effector_frame:
            print("Error: No end-effector frame found!")
            return
        
        iteration_count = 0
        print_interval = 20  # Print every 20 iterations (about 1 second at 50ms delay)
        
        # Log initial joint configuration
        initial_q = self.plant.GetPositions(self.plant_context)
        print(f"Initial joint configuration before IK:")
        for i, joint_name in enumerate(self.arm_joint_names):
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            print(f"  {joint_name}: {initial_q[joint_idx]:.4f} rad ({np.degrees(initial_q[joint_idx]):.2f} deg)")
        
        # Publish initial state if real robot mode
        if self.enable_real_robot:
            self.publish_joint_states()
            print("[IK Control] Published initial joint states to LCM")
        
        try:
            while True:
                iteration_count += 1
                # Read target pose from sliders
                target_x = self.meshcat.GetSliderValue("target_x")
                target_y = self.meshcat.GetSliderValue("target_y")
                target_z = self.meshcat.GetSliderValue("target_z")
                target_roll = self.meshcat.GetSliderValue("target_roll")
                target_pitch = self.meshcat.GetSliderValue("target_pitch")
                target_yaw = self.meshcat.GetSliderValue("target_yaw")
                gripper_value = self.meshcat.GetSliderValue("gripper")
                
                # Create target transform
                target_position = np.array([target_x, target_y, target_z])
                target_rotation = RotationMatrix(RollPitchYaw(target_roll, target_pitch, target_yaw))
                target_transform = RigidTransform(target_rotation, target_position)
                
                # Update target visualization
                self.meshcat.SetTransform("target_frame", target_transform)
                
                # Debug: Check if we're in a singularity or gimbal lock situation
                if iteration_count % print_interval == 0:
                    ee_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
                    ee_rpy = RollPitchYaw(ee_pose.rotation())
                    print(f"\n[DEBUG] Current EE: R={ee_rpy.roll_angle():.3f}, P={ee_rpy.pitch_angle():.3f}, Y={ee_rpy.yaw_angle():.3f}")
                    print(f"[DEBUG] Target:     R={target_roll:.3f}, P={target_pitch:.3f}, Y={target_yaw:.3f}")
                    
                    # Check joint configuration
                    q = self.plant.GetPositions(self.plant_context)
                    print(f"[DEBUG] Joint angles: {q[:6]}")
                
                # Use differential IK to move towards target
                result = DoDifferentialInverseKinematics(
                    self.plant,
                    self.plant_context,
                    target_transform,
                    self.end_effector_frame,
                    self.diff_ik_params
                )
                
                if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
                    # Get current positions
                    q_current = self.plant.GetPositions(self.plant_context)
                    
                    # Integrate velocities
                    v_sol = result.joint_velocities
                    q_new = q_current + v_sol.flatten() * self.dt
                    
                    # Set gripper separately (at its correct index)
                    try:
                        gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                        gripper_index = gripper_joint.position_start()
                        q_new[gripper_index] = gripper_value
                    except:
                        pass
                    
                    # Apply joint limits
                    q_lower = self.plant.GetPositionLowerLimits()
                    q_upper = self.plant.GetPositionUpperLimits()
                    q_new = np.clip(q_new, q_lower, q_upper)
                    
                    # Check if positions actually changed
                    position_tolerance = 1e-6
                    positions_changed = np.any(np.abs(q_new - q_current) > position_tolerance)
                    
                    # Set new positions
                    self.plant.SetPositions(self.plant_context, q_new)
                    
                    # Publish joint states to LCM only if positions changed and real robot mode is enabled
                    if positions_changed and self.enable_real_robot:
                        self.publish_joint_states()
                        # Only print first few times and then occasionally
                        if iteration_count < 5 or iteration_count % 100 == 0:
                            print(f"[IK Control] Published joint states to LCM")
                
                # Update gripper COM axes visualization
                if self.gripper_com_body:
                    gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.gripper_com_body
                    )
                    self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                    
                    # Update force vector visualization
                    self.update_force_vector()
                
                # Update OpenFT frame axes visualization
                if self.openft_body:
                    openft_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.openft_body
                    )
                    self.meshcat.SetTransform("openft_frame", openft_pose)
                
                # Calculate and print forces/torques periodically
                if iteration_count % print_interval == 0:
                    force_world, torque_world = self.calculate_openft_wrench()
                    if force_world is not None and self.openft_body:
                        print(f"\n--- OpenFT Forces & Torques ---")
                        print(f"WORLD FRAME:")
                        print(f"  Force:  [{force_world[0]:7.3f}, {force_world[1]:7.3f}, {force_world[2]:7.3f}] N")
                        print(f"  Torque: [{torque_world[0]:7.4f}, {torque_world[1]:7.4f}, {torque_world[2]:7.4f}] N⋅m")
                        
                        # Transform to OpenFT local frame
                        openft_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.openft_body)
                        R_world_to_openft = openft_pose.rotation().transpose()
                        force_openft = R_world_to_openft @ force_world
                        torque_openft = R_world_to_openft @ torque_world
                        
                        print(f"OPENFT LOCAL FRAME:")
                        print(f"  Force:  [{force_openft[0]:7.3f}, {force_openft[1]:7.3f}, {force_openft[2]:7.3f}] N")
                        print(f"  Torque: [{torque_openft[0]:7.4f}, {torque_openft[1]:7.4f}, {torque_openft[2]:7.4f}] N⋅m")
                        print(f"  Force magnitude:  {np.linalg.norm(force_openft):.3f} N")
                        print(f"  Torque magnitude: {np.linalg.norm(torque_openft):.4f} N⋅m")
                
                # Publish visualization
                self.diagram.ForcedPublish(self.diagram_context)
                
                # Small delay
                time.sleep(self.dt)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
    
    def run_calibration(self):
        """Run calibration sequence for OpenFT sensor."""
        import csv
        from datetime import datetime
        
        print("\n" + "="*60)
        print("OpenFT Calibration Sequence")
        print("="*60)
        print("This will move joint5 and joint6 through a calibration pattern")
        print("Recording forces, torques, and joint positions")
        if self.enable_sensor_collection:
            print("Sensor data collection: ENABLED (16 channels)")
        else:
            print("Sensor data collection: DISABLED (use --sensor to enable)")
        print("Step size: 0.1 radians, Delay: 500ms")
        print("="*60 + "\n")
        
        # 3-second countdown before starting
        print("Starting calibration in:")
        for i in range(3, 0, -1):
            print(f"  {i}...")
            time.sleep(1.0)
        print("Starting calibration now!\n")
        
        # Calibration parameters
        step_size = 0.1  # radians
        delay = 0.5  # 500ms
        
        # Data storage
        calibration_data = []
        
        # Initialize all joints to zero
        print("Step 1: Moving all joints to zero position...")
        initial_positions = np.zeros(self.plant.num_positions())
        self.plant.SetPositions(self.plant_context, initial_positions)
        self.publish_joint_states()  # Publish initial zero state
        self.diagram.ForcedPublish(self.diagram_context)
        time.sleep(1.0)  # Wait for settling
        
        # Get joint indices
        joint5 = self.plant.GetJointByName("joint5")
        joint6 = self.plant.GetJointByName("joint6")
        joint5_idx = joint5.position_start()
        joint6_idx = joint6.position_start()
        
        def record_data(step_name):
            """Record current state data."""
            # Get latest sensor data if available
            sensor_data = self.get_latest_sensor_data() if self.enable_sensor_collection else None
            
            # Get joint positions
            q = self.plant.GetPositions(self.plant_context)
            
            # Calculate forces and torques
            force_world, torque_world = self.calculate_openft_wrench()
            
            if force_world is not None and self.openft_body:
                # Transform to OpenFT local frame
                openft_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.openft_body)
                R_world_to_openft = openft_pose.rotation().transpose()
                force_local = R_world_to_openft @ force_world
                torque_local = R_world_to_openft @ torque_world
                
                # Record data
                data_point = {
                    'timestamp': time.time(),
                    'step': step_name,
                    'joint1': q[0], 'joint2': q[1], 'joint3': q[2],
                    'joint4': q[3], 'joint5': q[4], 'joint6': q[5],
                    'force_local_x': force_local[0],
                    'force_local_y': force_local[1],
                    'force_local_z': force_local[2],
                    'torque_local_x': torque_local[0],
                    'torque_local_y': torque_local[1],
                    'torque_local_z': torque_local[2],
                    'force_world_x': force_world[0],
                    'force_world_y': force_world[1],
                    'force_world_z': force_world[2],
                    'torque_world_x': torque_world[0],
                    'torque_world_y': torque_world[1],
                    'torque_world_z': torque_world[2],
                }
                
                # Add sensor data if available
                if sensor_data and 'sensor_moving_averages' in sensor_data:
                    sensor_values = sensor_data['sensor_moving_averages']
                    # Add all 16 sensor values
                    for i, val in enumerate(sensor_values):
                        data_point[f'sensor_{i+1}'] = val
                    # Also add sensor timestamp if available
                    if 'timestamp' in sensor_data:
                        data_point['sensor_timestamp'] = sensor_data['timestamp']
                
                calibration_data.append(data_point)
                
                # Print current state
                print(f"[{step_name}] J5: {q[4]:.3f}, J6: {q[5]:.3f} | "
                      f"F_local: [{force_local[0]:.2f}, {force_local[1]:.2f}, {force_local[2]:.2f}] | "
                      f"T_local: [{torque_local[0]:.3f}, {torque_local[1]:.3f}, {torque_local[2]:.3f}]")
        
        # Record initial state
        record_data("initial")
        
        # Step 2: Move joint5 to -π/2
        print("\nStep 2: Moving joint5 to -π/2...")
        current_j5 = 0.0
        target_j5 = -np.pi/2
        
        while current_j5 > target_j5:
            current_j5 -= step_size
            if current_j5 < target_j5:
                current_j5 = target_j5
            
            q = self.plant.GetPositions(self.plant_context)
            q[joint5_idx] = current_j5
            self.plant.SetPositions(self.plant_context, q)
            
            # Publish joint states to LCM if real robot mode is enabled
            self.publish_joint_states()
            
            # Update visualizations
            if self.gripper_com_body:
                gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.gripper_com_body
                )
                self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                self.update_force_vector()
            
            if self.openft_body:
                openft_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.openft_body
                )
                self.meshcat.SetTransform("openft_frame", openft_pose)
            
            self.diagram.ForcedPublish(self.diagram_context)
            time.sleep(delay)  # Let sensor settle
            record_data(f"j5_to_-pi/2")  # Record after settling
        
        # Step 3: Move joint6 from 0 to -π (keeping joint5 at -π/2)
        print("\nStep 3: Moving joint6 to -π (joint5 at -π/2)...")
        current_j6 = 0.0
        target_j6 = -np.pi
        
        while current_j6 > target_j6:
            current_j6 -= step_size
            if current_j6 < target_j6:
                current_j6 = target_j6
            
            q = self.plant.GetPositions(self.plant_context)
            q[joint6_idx] = current_j6
            self.plant.SetPositions(self.plant_context, q)
            
            # Publish joint states to LCM if real robot mode is enabled
            self.publish_joint_states()
            
            # Update visualizations
            if self.gripper_com_body:
                gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.gripper_com_body
                )
                self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                self.update_force_vector()
            
            if self.openft_body:
                openft_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.openft_body
                )
                self.meshcat.SetTransform("openft_frame", openft_pose)
            
            self.diagram.ForcedPublish(self.diagram_context)
            time.sleep(delay)  # Let sensor settle
            record_data(f"j6_to_-pi")  # Record after settling
        
        # Step 4: Move joint6 from -π to +π (keeping joint5 at -π/2)
        print("\nStep 4: Moving joint6 to +π (joint5 at -π/2)...")
        target_j6 = np.pi
        
        while current_j6 < target_j6:
            current_j6 += step_size
            if current_j6 > target_j6:
                current_j6 = target_j6
            
            q = self.plant.GetPositions(self.plant_context)
            q[joint6_idx] = current_j6
            self.plant.SetPositions(self.plant_context, q)
            
            # Publish joint states to LCM if real robot mode is enabled
            self.publish_joint_states()
            
            # Update visualizations
            if self.gripper_com_body:
                gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.gripper_com_body
                )
                self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                self.update_force_vector()
            
            if self.openft_body:
                openft_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.openft_body
                )
                self.meshcat.SetTransform("openft_frame", openft_pose)
            
            self.diagram.ForcedPublish(self.diagram_context)
            time.sleep(delay)  # Let sensor settle
            record_data(f"j6_to_+pi")  # Record after settling
        
        # Step 5: Move joint6 back to 0 (keeping joint5 at -π/2)
        print("\nStep 5: Moving joint6 back to 0 (joint5 at -π/2)...")
        target_j6 = 0.0
        
        while current_j6 > target_j6:
            current_j6 -= step_size
            if current_j6 < target_j6:
                current_j6 = target_j6
            
            q = self.plant.GetPositions(self.plant_context)
            q[joint6_idx] = current_j6
            self.plant.SetPositions(self.plant_context, q)
            
            # Publish joint states to LCM if real robot mode is enabled
            self.publish_joint_states()
            
            # Update visualizations
            if self.gripper_com_body:
                gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.gripper_com_body
                )
                self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                self.update_force_vector()
            
            if self.openft_body:
                openft_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.openft_body
                )
                self.meshcat.SetTransform("openft_frame", openft_pose)
            
            self.diagram.ForcedPublish(self.diagram_context)
            time.sleep(delay)  # Let sensor settle
            record_data(f"j6_to_0")  # Record after settling
        
        # Step 6: Move joint5 back to 0 (joint6 at 0)
        print("\nStep 6: Moving joint5 back to 0...")
        current_j5 = -np.pi/2
        target_j5 = 0.0
        
        while current_j5 < target_j5:
            current_j5 += step_size
            if current_j5 > target_j5:
                current_j5 = target_j5
            
            q = self.plant.GetPositions(self.plant_context)
            q[joint5_idx] = current_j5
            self.plant.SetPositions(self.plant_context, q)
            
            # Publish joint states to LCM if real robot mode is enabled
            self.publish_joint_states()
            
            # Update visualizations
            if self.gripper_com_body:
                gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.gripper_com_body
                )
                self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                self.update_force_vector()
            
            if self.openft_body:
                openft_pose = self.plant.EvalBodyPoseInWorld(
                    self.plant_context, self.openft_body
                )
                self.meshcat.SetTransform("openft_frame", openft_pose)
            
            self.diagram.ForcedPublish(self.diagram_context)
            time.sleep(delay)  # Let sensor settle
            record_data(f"j5_to_0")  # Record after settling
        
        # Save calibration data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"openft_calibration_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as csvfile:
            if calibration_data:
                fieldnames = calibration_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(calibration_data)
        
        print(f"\n{'='*60}")
        print(f"Calibration complete!")
        print(f"Data saved to: {filename}")
        print(f"Total data points: {len(calibration_data)}")
        print(f"{'='*60}\n")
    
    def run(self):
        """Main run method."""
        if self.use_ik_control:
            self.run_ik_control()
        else:
            self.run_joint_control()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Drake xARM OpenFT Control Test")
    parser.add_argument(
        "--ik_control",
        action="store_true",
        help="Use inverse kinematics control mode (default: joint control)"
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run calibration sequence for OpenFT sensor"
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="Enable real robot control via LCM joint state publishing"
    )
    parser.add_argument(
        "--sensor",
        action="store_true",
        help="Enable sensor data collection via ZMQ (port 5555)"
    )
    parser.add_argument(
        "--xarm",
        type=str,
        help="xARM IP address to read initial joint positions (e.g., 192.168.1.210)"
    )
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Drake xARM6 OpenFT Gripper Control")
    if args.real:
        print("Real Robot Mode: ENABLED - Publishing to LCM")
    if args.sensor:
        print("Sensor Collection: ENABLED - Subscribing to ZMQ port 5555")
    if args.xarm:
        print(f"xARM Sync: ENABLED - Reading positions from {args.xarm}")
    print("="*60)
    
    # Handle calibration mode
    if args.calibrate:
        controller = XArmOpenFTController(
            use_ik_control=False, 
            enable_real_robot=args.real,
            enable_sensor_collection=args.sensor,
            xarm_ip=args.xarm
        )
        controller.run_calibration()
    else:
        # Create and run controller
        controller = XArmOpenFTController(
            use_ik_control=args.ik_control, 
            enable_real_robot=args.real,
            enable_sensor_collection=args.sensor,
            xarm_ip=args.xarm
        )
        controller.run()


if __name__ == "__main__":
    main()
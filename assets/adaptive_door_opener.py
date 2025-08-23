#!/usr/bin/env python3
"""
Adaptive Door Opening Controller using Force-Torque Feedback

This controller uses force-torque sensor readings to adaptively open articulated objects
like microwave doors. It implements a reactive control policy that:
1. Reads force-torque sensor data to detect contact forces
2. Computes desired motion direction to reduce reactive forces
3. Uses differential IK to move the end-effector along computed trajectories
"""

import numpy as np
import argparse
import time
import lcm
import zmq
import json
from collections import deque
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
    DoDifferentialInverseKinematics,
    DifferentialInverseKinematicsStatus,
    DifferentialInverseKinematicsParameters,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)
import os
from xarm.wrapper import XArmAPI


class AdaptiveDoorOpener:
    def __init__(self, xarm_ip=None, enable_real_robot=True, force_scale=0.01, force_axes=None, pull_weight=1.0):
        """
        Initialize the adaptive door opener controller.
        
        Args:
            xarm_ip: IP address of the xARM robot
            enable_real_robot: Whether to publish commands to real robot via LCM
            force_scale: Scale factor for force-based motion (m/N)
            force_axes: List of axes to consider for force feedback ['x', 'y', 'z']
            pull_weight: Weight for pulling component in -z direction (0=no pull, 1=normal, 2=strong)
        """

        # Start meshcat
        self.meshcat = StartMeshcat()
        
        # Control parameters
        self.xarm_ip = xarm_ip
        self.enable_real_robot = enable_real_robot
        self.force_scale = force_scale  # How much to move per Newton of force
        self.motion_step_size = 0.02  # 2cm movements
        self.force_threshold = 1.0  # Minimum force magnitude to trigger motion (N)
        self.torque_threshold = 0.01  # Minimum torque magnitude (N⋅m)
        self.pull_weight = pull_weight  # Weight for pulling component
        
        # Force axes configuration
        self.force_axes = force_axes if force_axes else ['x', 'y', 'z']
        self.force_mask = np.array([
            1.0 if 'x' in self.force_axes else 0.0,
            1.0 if 'y' in self.force_axes else 0.0,
            1.0 if 'z' in self.force_axes else 0.0
        ])
        print(f"Using force axes: {self.force_axes}")
        print(f"Force mask: {self.force_mask}")
        
        # Force-torque data storage
        self.force_history = deque(maxlen=10)  # Store last 10 readings
        self.torque_history = deque(maxlen=10)
        self.baseline_force = None
        self.baseline_torque = None
        self.current_force = np.zeros(3)
        self.current_torque = np.zeros(3)
        
        # Motion planning
        self.target_pose = None
        self.motion_direction = np.array([0, 0, 0])
        self.is_first_move = True
        self.is_first_target = True  # Track first target computation separately
        
        # Get initial xARM positions
        self.xarm_initial_positions = None
        if self.xarm_ip:
            print(f"\n{'='*60}")
            print(f"Connecting to xARM at {self.xarm_ip}")
            print(f"{'='*60}")
            self.xarm_initial_positions = self.get_xarm_positions()
            if self.xarm_initial_positions is None:
                print("WARNING: Failed to get xARM positions, using default")
                print(f"{'='*60}\n")
            
        self.arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position control mode
        self.arm.set_state(0)  # Set to ready state
        
        # Initialize LCM for robot control
        if self.enable_real_robot:
            self.lc = lcm.LCM()
            self.joint_state_msg = JointState()
            print("LCM initialized for real robot control")
        
        # Initialize ZMQ for force-torque data
        self.setup_force_torque_receiver()
        
        # Setup Drake simulation
        self.setup_simulation()
        
        # Setup differential IK
        self.setup_diff_ik()
        
        # Create visualizations
        self.create_force_visualizations()
        
        # Initial publish
        self.diagram.ForcedPublish(self.diagram_context)
        
        print(f"\n{'='*60}")
        print(f"Meshcat URL: {self.meshcat.web_url()}")
        print(f"Open this URL in your browser to view the robot")
        print(f"{'='*60}\n")
    
    def get_xarm_positions(self):
        """Get current joint positions from xARM robot."""
        try:
            from xarm.wrapper import XArmAPI
            
            print(f"Connecting to xARM...")
            arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            
            # Clear any errors
            arm.clean_error()
            arm.clean_warn()
            
            # Get current joint angles (6 DOF)
            code, angles = arm.get_servo_angle(is_radian=True)
            
            if code == 0 and angles:
                print(f"Got xARM joint positions:")
                for i, angle in enumerate(angles[:6]):
                    print(f"  joint{i+1}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")
                
                # Try to get gripper position
                try:
                    code_gripper, gripper_pos = arm.get_gripper_position()
                    if code_gripper == 0:
                        # Convert gripper position to radians for drive_joint
                        # xARM gripper position is typically in mm, need to convert
                        # to drive_joint radians (0 = closed, 0.85 = open)
                        gripper_rad = gripper_pos / 1000.0  # Rough conversion
                        print(f"  gripper: {gripper_pos:.1f} mm (~{gripper_rad:.3f} rad)")
                        # Return 7 values: 6 joints + gripper
                        result = list(angles[:6])
                        result.append(gripper_rad)
                        arm.disconnect()
                        return result
                except:
                    # If gripper read fails, just return 6 joint angles
                    pass
                
                arm.disconnect()
                return angles[:6]
            else:
                print(f"Failed to get xARM positions, code: {code}")
            
            arm.disconnect()
            
        except ImportError:
            print("Error: xarm library not installed")
        except Exception as e:
            print(f"Error connecting to xARM: {e}")
        
        return None
    
    def setup_force_torque_receiver(self):
        """Setup ZMQ subscriber for force-torque data."""
        try:
            self.zmq_context = zmq.Context()
            self.ft_socket = self.zmq_context.socket(zmq.SUB)
            self.ft_socket.connect("tcp://localhost:5556")  # Port for calibrated data
            self.ft_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self.ft_socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout
            print("Connected to force-torque data stream on port 5556")
        except Exception as e:
            print(f"Warning: Failed to setup force-torque receiver: {e}")
            self.ft_socket = None
    
    def get_force_torque_data(self):
        """Get latest force-torque sensor data."""
        if not self.ft_socket:
            return None
        
        latest_data = None
        try:
            # Drain socket to get latest message
            while True:
                try:
                    data_str = self.ft_socket.recv_string(zmq.NOBLOCK)
                    data = json.loads(data_str)
                    if 'forces' in data and 'torques' in data:
                        latest_data = data
                except zmq.Again:
                    break
                except json.JSONDecodeError as e:
                    print(f"Error decoding force-torque JSON: {e}")
        except Exception as e:
            print(f"Error getting force-torque data: {e}")
        
        if latest_data:
            self.current_force = np.array(latest_data['forces'])
            self.current_torque = np.array(latest_data['torques'])
            self.force_history.append(self.current_force.copy())
            self.torque_history.append(self.current_torque.copy())
        
        return latest_data
    
    def setup_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()
        
        # Create diagram builder
        self.builder = DiagramBuilder()
        
        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.001
        )
        
        # Parse URDF
        parser = Parser(self.plant)
        package_path = os.path.dirname(os.path.abspath(__file__))
        parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))
        
        # Load the URDF
        urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
        self.model_instances = parser.AddModels(urdf_path)
        self.model_instance = self.model_instances[0] if self.model_instances else None
        
        # Get link_openft frame for control
        try:
            self.openft_frame = self.plant.GetFrameByName("link_openft")
            self.openft_body = self.plant.GetBodyByName("link_openft")
            print("Using link_openft as control frame")
        except:
            print("ERROR: Could not find link_openft frame!")
            raise
        
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
        
        # Store gripper position separately
        self.gripper_position = None
        
        # Use xARM positions if available
        if self.xarm_initial_positions is not None:
            print("\nInitializing Drake with xARM joint positions")
            for i, joint_name in enumerate(self.arm_joint_names):
                try:
                    joint = self.plant.GetJointByName(joint_name)
                    joint_index = joint.position_start()
                    if i < len(self.xarm_initial_positions):
                        initial_positions[joint_index] = self.xarm_initial_positions[i]
                except Exception as e:
                    print(f"  Error setting {joint_name}: {e}")
            
            # Check if we have a 7th value for the gripper
            if len(self.xarm_initial_positions) > 6:
                self.gripper_position = self.xarm_initial_positions[6]
                print(f"  Got gripper position from xARM: {self.gripper_position:.3f}")
        
        # Set gripper position (use xARM value if available, otherwise default)
        try:
            gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
            gripper_index = gripper_joint.position_start()
            if self.gripper_position is not None:
                initial_positions[gripper_index] = self.gripper_position
            else:
                # Default to slightly closed if no xARM data
                self.gripper_position = 0.02
                initial_positions[gripper_index] = self.gripper_position
                print(f"  Using default gripper position: {self.gripper_position:.3f}")
        except:
            pass
        
        self.plant.SetPositions(self.plant_context, initial_positions)
        
        # Store initial openft pose
        self.initial_openft_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        
        # Reset camera
        self.meshcat.SetCameraPose(
            camera_in_world=[1.5, 1.5, 1.2],
            target_in_world=[0.0, 0.0, 0.3]
        )
    
    def setup_diff_ik(self):
        """Setup differential IK parameters."""
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
        
        # Enable all 6 DOF for end-effector control
        velocity_flag = np.ones(6, dtype=bool)
        self.diff_ik_params.set_end_effector_velocity_flag(velocity_flag)
    
    def create_force_visualizations(self):
        """Create visualizations for forces and motion directions."""
        # Reactive force vector (what sensor measures)
        self.meshcat.SetObject(
            "reactive_force/arrow",
            Cylinder(0.005, 0.1),  # Will be scaled dynamically
            Rgba(1.0, 0.0, 0.0, 0.8)  # Red for reactive force
        )
        self.meshcat.SetObject(
            "reactive_force/head",
            Sphere(0.015),
            Rgba(1.0, 0.0, 0.0, 0.9)
        )
        
        # Desired motion direction (opposite of reactive force in x,y)
        self.meshcat.SetObject(
            "motion_direction/arrow",
            Cylinder(0.006, 0.1),
            Rgba(0.0, 1.0, 0.0, 0.8)  # Green for desired motion
        )
        self.meshcat.SetObject(
            "motion_direction/head",
            Sphere(0.018),
            Rgba(0.0, 1.0, 0.0, 0.9)
        )
        
        # Target pose visualization
        self.create_target_frame_visualization()
        
        print("Created force and motion visualizations")
    
    def create_target_frame_visualization(self):
        """Create visualization for target pose."""
        axis_length = 0.12
        axis_radius = 0.004
        
        # X-axis (red)
        self.meshcat.SetObject(
            "target_pose/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(0.8, 0.2, 0.2, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green)
        self.meshcat.SetObject(
            "target_pose/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.2, 0.8, 0.2, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue)
        self.meshcat.SetObject(
            "target_pose/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.2, 0.2, 0.8, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
    
    def update_force_visualizations(self):
        """Update force vector visualizations based on current readings."""
        # Get current openft pose
        openft_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        openft_pos = openft_pose.translation()
        openft_rot = openft_pose.rotation()
        
        # Calculate force difference from baseline (if available)
        if self.baseline_force is not None:
            force_diff = self.current_force - self.baseline_force
        else:
            force_diff = self.current_force
        
        # Transform force from sensor frame to world frame
        force_world = openft_rot @ force_diff
        
        # Visualize reactive force (scale: 0.02 m per Newton)
        force_scale = 0.02
        force_magnitude = np.linalg.norm(force_world)
        
        if force_magnitude > 0.1:  # Only show if significant
            force_direction = force_world / force_magnitude
            force_length = force_magnitude * force_scale
            
            # Position arrow from openft position
            arrow_center = openft_pos + force_direction * force_length / 2
            
            # Rotation to align cylinder with force direction
            z_axis = np.array([0, 0, 1])
            if not np.allclose(force_direction, z_axis):
                axis = np.cross(z_axis, force_direction)
                axis = axis / np.linalg.norm(axis)
                angle = np.arccos(np.dot(z_axis, force_direction))
                # Create rotation matrix using Rodrigues' formula
                K = np.array([[0, -axis[2], axis[1]],
                             [axis[2], 0, -axis[0]],
                             [-axis[1], axis[0], 0]])
                R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                rotation = RotationMatrix(R)
            else:
                rotation = RotationMatrix()
            
            # Update reactive force visualization
            self.meshcat.SetTransform(
                "reactive_force/arrow",
                RigidTransform(rotation, arrow_center)
            )
            
            # Position arrow head
            head_position = openft_pos + force_direction * force_length
            self.meshcat.SetTransform(
                "reactive_force/head",
                RigidTransform(RotationMatrix(), head_position)
            )
        
        # Visualize desired motion direction
        if not np.allclose(self.motion_direction, 0):
            motion_magnitude = np.linalg.norm(self.motion_direction)
            motion_unit = self.motion_direction / motion_magnitude
            motion_length = self.motion_step_size * 5  # Show 5x actual step for visibility
            
            # Position arrow from openft position
            arrow_center = openft_pos + motion_unit * motion_length / 2
            
            # Rotation to align cylinder with motion direction
            z_axis = np.array([0, 0, 1])
            if not np.allclose(motion_unit, z_axis):
                axis = np.cross(z_axis, motion_unit)
                axis = axis / np.linalg.norm(axis)
                angle = np.arccos(np.dot(z_axis, motion_unit))
                K = np.array([[0, -axis[2], axis[1]],
                             [axis[2], 0, -axis[0]],
                             [-axis[1], axis[0], 0]])
                R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                rotation = RotationMatrix(R)
            else:
                rotation = RotationMatrix()
            
            # Update motion direction visualization
            self.meshcat.SetTransform(
                "motion_direction/arrow",
                RigidTransform(rotation, arrow_center)
            )
            
            # Position arrow head
            head_position = openft_pos + motion_unit * motion_length
            self.meshcat.SetTransform(
                "motion_direction/head",
                RigidTransform(RotationMatrix(), head_position)
            )
    
    def compute_motion_direction(self):
        """
        Compute desired motion direction based on force-torque readings.
        
        Returns:
            np.array: 3D motion direction in world frame
        """
        # Get force difference from baseline
        if self.baseline_force is not None:
            force_diff = self.current_force - self.baseline_force
            torque_diff = self.current_torque - self.baseline_torque
        else:
            force_diff = self.current_force
            torque_diff = self.current_torque
        
        # Apply force mask to only consider specified axes
        force_diff = force_diff * self.force_mask
        
        # Get current openft rotation to transform from sensor to world frame
        openft_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        openft_rot = openft_pose.rotation()
        
        # Transform force from sensor frame to world frame
        force_world = openft_rot @ force_diff
        
        # For first move, go straight in -z direction (openft frame)
        if self.is_first_move:
            print("First move: Moving in -z direction (openft frame)")
            # Create motion in openft local frame (-z direction)
            motion_local_first = np.array([0.0, 0.0, -1.0])
            # Transform to world frame
            self.motion_direction = openft_rot @ motion_local_first
            print(f"  Motion direction in world frame: {self.motion_direction}")
            self.is_first_move = False
            return self.motion_direction
        
        # Compute motion direction to reduce reactive forces
        # Invert x and y components (move opposite to reactive force)
        # Add pulling component in -z direction of openft frame
        # TODO: inspect whether this is right
        
        # Separate compliance and pulling components
        # Limit compliance magnitude to prevent overwhelming pulling
        max_compliance_per_axis = 0.5  # Maximum compliance contribution
        
        compliance_component = np.array([
            np.clip(-force_diff[0] * 0.1, -max_compliance_per_axis, max_compliance_per_axis) if 'x' in self.force_axes else 0.0,
            np.clip(-force_diff[1] * 0.1, -max_compliance_per_axis, max_compliance_per_axis) if 'y' in self.force_axes else 0.0,
            0.0
        ])
        
        # Strong pulling component that dominates
        pulling_component = np.array([0.0, 0.0, -3.0 * self.pull_weight])  # Strong pulling
        
        # Combine - pulling should dominate
        motion_local = compliance_component + pulling_component
        
        # Debug output
        print(f"  Force diff (local): {force_diff}")
        print(f"  Compliance component: {compliance_component}")
        print(f"  Pulling component: {pulling_component}")
        print(f"  Combined motion (local): {motion_local}")
        
        # Normalize the local motion vector
        motion_local_magnitude = np.linalg.norm(motion_local)
        if motion_local_magnitude > 0.01:
            motion_local = motion_local / motion_local_magnitude
        else:
            # If motion is too small, default to pure pulling
            motion_local = np.array([0.0, 0.0, -1.0])
            print("  Motion too small, defaulting to pure pull")
        
        print(f"  Normalized motion (local): {motion_local}")
        
        # Transform to world frame
        motion_world = openft_rot @ motion_local
        print(f"  Motion (world): {motion_world}")
        
        # Add torque-based adjustments (simplified)
        # If there's significant torque, add a small rotational component
        if np.linalg.norm(torque_diff) > self.torque_threshold:
            # This is simplified - in practice you'd want more sophisticated torque handling
            torque_world = openft_rot @ torque_diff
            # Add small component based on torque (cross product gives perpendicular direction)
            torque_adjustment = np.cross(torque_world, np.array([0, 0, 1])) * 0.1
            motion_world += torque_adjustment
        
        # Always set motion direction (we've already normalized in local frame)
        self.motion_direction = motion_world
        
        # Don't check force threshold since we always want to pull
        # The pulling component ensures we always have motion
        
        # Store for next iteration
        self.previous_direction = self.motion_direction.copy()
        
        return self.motion_direction
    
    def compute_target_pose(self):
        """
        Compute target pose for differential IK based on motion direction.
        
        Returns:
            RigidTransform: Target pose for link_openft
        """
        # Get current openft pose
        current_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        current_pos = current_pose.translation()
        current_rot = current_pose.rotation()
        
        # Compute target position (1cm in motion direction)
        target_position = current_pos + self.motion_direction * self.motion_step_size
        
        # For first move, keep orientation unchanged (pure translation)
        if hasattr(self, 'is_first_target') and self.is_first_target:
            target_rotation = current_rot
            self.is_first_target = False
            print("First move: Pure translation, no rotation")
        # Compute target orientation to align with motion direction
        # We want to rotate the gripper to face the direction of motion
        elif np.linalg.norm(self.motion_direction) > 0.1:
            # Get current forward direction (x-axis of openft frame)
            current_x_axis = current_rot.matrix()[:, 0]
            
            # Project motion direction onto horizontal plane for orientation
            motion_horizontal = self.motion_direction.copy()
            motion_horizontal[2] = 0  # Remove z component for yaw calculation
            
            if np.linalg.norm(motion_horizontal) > 0.01:
                # Calculate desired yaw to align with motion
                motion_normalized = motion_horizontal / np.linalg.norm(motion_horizontal)
                
                # We want the gripper's -z axis (pulling direction) to align with motion
                # Get current -z axis of openft frame (pulling direction)
                current_z_axis = -current_rot.matrix()[:, 2]  # Negative z is pulling direction
                current_z_horizontal = current_z_axis.copy()
                current_z_horizontal[2] = 0
                
                if np.linalg.norm(current_z_horizontal) > 0.01:
                    current_z_horizontal = current_z_horizontal / np.linalg.norm(current_z_horizontal)
                    
                    # Compute rotation angle to align -z with motion
                    rotation_gain = 0.5  # Rotate 50% of the way toward the target
                    dot_product = np.clip(np.dot(current_z_horizontal, motion_normalized), -1, 1)
                    angle = np.arccos(dot_product)
                    
                    # Determine rotation direction using cross product
                    cross = np.cross(current_z_horizontal, motion_normalized)
                    if cross[2] < 0:
                        angle = -angle
                    
                    # Apply partial rotation
                    angle = angle * rotation_gain
                    
                    # Create rotation matrix for yaw adjustment
                    c = np.cos(angle)
                    s = np.sin(angle)
                    yaw_rotation = np.array([
                        [c, -s, 0],
                        [s, c, 0],
                        [0, 0, 1]
                    ])
                    
                    # Apply yaw rotation to current rotation
                    target_rotation = RotationMatrix(yaw_rotation @ current_rot.matrix())
                else:
                    target_rotation = current_rot
            else:
                target_rotation = current_rot
        else:
            # No significant motion, keep current orientation
            target_rotation = current_rot
        
        self.target_pose = RigidTransform(target_rotation, target_position)
        
        # Update target visualization
        self.meshcat.SetTransform("target_pose", self.target_pose)
        
        return self.target_pose
    
    def execute_motion_step(self):
        """Execute one motion step using differential IK."""
        if self.target_pose is None:
            return False
        
        # Use differential IK to move towards target
        result = DoDifferentialInverseKinematics(
            self.plant,
            self.plant_context,
            self.target_pose,
            self.openft_frame,
            self.diff_ik_params
        )
        
        if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
            # Get current positions
            q_current = self.plant.GetPositions(self.plant_context)
            
            # Integrate velocities
            v_sol = result.joint_velocities
            q_new = q_current + v_sol.flatten() * self.dt
            
            # Apply joint limits
            q_lower = self.plant.GetPositionLowerLimits()
            q_upper = self.plant.GetPositionUpperLimits()
            q_new = np.clip(q_new, q_lower, q_upper)
            
            # Preserve gripper position (don't let IK change it)
            try:
                gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                gripper_index = gripper_joint.position_start()
                q_new[gripper_index] = self.gripper_position
            except Exception as e:
                print(f"Warning: Gripper joint not found: {e}")
            
            # Set new positions
            self.plant.SetPositions(self.plant_context, q_new)
            
            # Publish to real robot if enabled
            if self.enable_real_robot:
                # self.publish_joint_states()
                self.command_xarm()
            
            return True
        else:
            print(f"IK failed with status: {result.status}")
            return False
    
    def command_xarm(self):
        if not self.enable_real_robot:
            return
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.set_state(0)
        self.arm.set_mode(0)
        
        # Get current joint positions
        q = self.plant.GetPositions(self.plant_context)
        
        # Prepare JointState message
        self.joint_state_msg.header.stamp.sec = int(time.time())
        self.joint_state_msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        self.joint_state_msg.header.frame_id = "base_link"
        
        # Set joint names and positions (arm joints only, no gripper)
        self.joint_state_msg.name = self.arm_joint_names.copy()
        self.joint_state_msg.position = []
        
        positions = []
        # Get positions for each arm joint
        for joint_name in self.arm_joint_names:
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            self.joint_state_msg.position.append(q[joint_idx])
            positions.append(q[joint_idx])

        code = self.arm.set_servo_angle(angle=positions, speed=0.05, wait=True, is_radian=True)
        if code != 0:
            print(f"Error commanding xARM: code {code}")
            return
        else:
            print(f"Moved to angles: {positions}")

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
        
        # Set joint names and positions (arm joints only, no gripper)
        self.joint_state_msg.name = self.arm_joint_names.copy()
        self.joint_state_msg.position = []
        
        # Get positions for each arm joint
        for joint_name in self.arm_joint_names:
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            self.joint_state_msg.position.append(q[joint_idx])
        
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
    
    def run(self):
        """Main control loop for adaptive door opening."""
        print("\n" + "="*60)
        print("Adaptive Door Opening Controller")
        print("="*60)
        print("The robot will use force-torque feedback to adaptively open the door")
        print("First move: 1cm in -x direction (world frame)")
        print("Subsequent moves: Based on force-torque feedback")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        # Wait for initial force-torque data
        print("Waiting for force-torque sensor data...")
        while self.get_force_torque_data() is None:
            time.sleep(0.1)
        
        # Set baseline forces (current reading with door handle gripped)
        print("Setting baseline force-torque readings...")
        time.sleep(1.0)  # Let readings stabilize
        self.get_force_torque_data()
        self.baseline_force = self.current_force.copy()
        self.baseline_torque = self.current_torque.copy()
        print(f"Baseline force: {self.baseline_force}")
        print(f"Baseline torque: {self.baseline_torque}")
        
        iteration = 0
        motion_count = 0
        max_motions = 100  # Safety limit
        
        try:
            while motion_count < max_motions:
                iteration += 1
                
                # Get latest force-torque data
                ft_data = self.get_force_torque_data()
                
                if ft_data is not None:
                    # Store force before movement
                    force_before = self.current_force.copy()
                    torque_before = self.current_torque.copy()
                    
                    # Compute motion direction
                    self.compute_motion_direction()
                    
                    # Update visualizations
                    self.update_force_visualizations()
                    
                    # Check if we should move - always move after computing direction
                    if np.linalg.norm(self.motion_direction) > 0.01:  # Lower threshold
                        # Compute target pose
                        self.compute_target_pose()
                        
                        # Execute motion
                        print(f"\n[Motion {motion_count+1}]")
                        masked_force_diff = (self.current_force - self.baseline_force) * self.force_mask
                        print(f"  Force diff (raw): {self.current_force - self.baseline_force}")
                        print(f"  Force diff (masked): {masked_force_diff}")
                        print(f"  Active axes: {self.force_axes}")
                        print(f"  Motion dir (magnitude={np.linalg.norm(self.motion_direction):.3f}): {self.motion_direction}")
                        
                        success = self.execute_motion_step()
                        
                        if success:
                            motion_count += 1
                            
                            # Wait for motion to complete and forces to stabilize
                            time.sleep(15.0)  # 3 second pause between motions
                            
                            # Get force after movement
                            self.get_force_torque_data()
                            force_after = self.current_force.copy()
                            torque_after = self.current_torque.copy()
                            
                            # Calculate force change
                            force_change = force_after - force_before
                            torque_change = torque_after - torque_before
                            
                            print(f"  Force change: {force_change}")
                            print(f"  Force magnitude change: {np.linalg.norm(force_after) - np.linalg.norm(force_before):.3f} N")
                            
                            # Update baseline if force reduced significantly
                            if np.linalg.norm(force_after) < np.linalg.norm(force_before) - 0.5:
                                print("  Force reduced - updating baseline")
                                self.baseline_force = force_after.copy()
                                self.baseline_torque = torque_after.copy()
                        else:
                            print("  Motion failed - IK solution not found")
                    else:
                        # No significant forces detected
                        if iteration % 20 == 0:
                            print(f"Iteration {iteration}: No significant forces detected")
                
                # Update visualization
                self.diagram.ForcedPublish(self.diagram_context)
                
                # Control loop rate
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nStopping adaptive door opener...")
        
        print(f"\nCompleted {motion_count} motion steps")
        print("Shutting down...")
        
        # Cleanup
        if self.ft_socket:
            self.ft_socket.close()
            self.zmq_context.term()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Adaptive Door Opening using Force-Torque Feedback"
    )
    parser.add_argument(
        "--xarm",
        type=str,
        help="xARM IP address to read initial joint positions"
    )
    parser.add_argument(
        "--sim_only",
        action="store_true",
        help="Run in simulation only (don't publish to real robot)"
    )
    parser.add_argument(
        "--force_scale",
        type=float,
        default=0.01,
        help="Scale factor for force-based motion (m/N)"
    )
    parser.add_argument(
        "--x",
        action="store_true",
        help="Use force feedback from X axis"
    )
    parser.add_argument(
        "--y",
        action="store_true",
        help="Use force feedback from Y axis"
    )
    parser.add_argument(
        "--z",
        action="store_true",
        help="Use force feedback from Z axis"
    )
    parser.add_argument(
        "--pull_weight",
        type=float,
        default=1.0,
        help="Weight for pulling component (0=no pull, 1=normal, 2=strong)"
    )
    args = parser.parse_args()
    
    # Determine which force axes to use
    force_axes = []
    if args.x:
        force_axes.append('x')
    if args.y:
        force_axes.append('y')
    if args.z:
        force_axes.append('z')
    
    # If no axes specified, default to x and y
    if not force_axes:
        force_axes = ['x', 'y']
        print("No force axes specified, defaulting to X and Y axes")
    
    # Create and run controller
    controller = AdaptiveDoorOpener(
        xarm_ip=args.xarm,
        enable_real_robot=not args.sim_only,
        force_scale=args.force_scale,
        force_axes=force_axes,
        pull_weight=args.pull_weight
    )
    controller.run()


if __name__ == "__main__":
    main()
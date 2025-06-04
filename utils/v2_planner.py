#!/usr/bin/env python3
"""
Planner that incorporates convex-hull meshes from RGB-to-Convex pipeline
as collision geometries, and uses inverse kinematics to generate a feasible
goal configuration before running OMPL RRT* planning.

1. Builds a Drake MultibodyPlant with the robot URDF.
2. Calls `run_pipeline()` to compute convex-hull meshes (Open3D TriangleMesh objects).
3. Converts each mesh into a Drake-compatible .obj via trimesh, then registers it
   as collision and visual geometry.
4. Provides an `inverse_kinematics()` method (borrowed from Kinematics class) to solve
   for a joint configuration that places the end-effector at a target pose.
5. Uses that IK result as the goal `q_goal` for OMPL RRT* planning.
6. Visualizes robot and objects in Meshcat.
7. Adds a new `forward_kinematics()` method to compute end-effector pose from joint angles.
"""

import os
import sys
import struct
import time
import argparse
import threading
from threading import Thread
from typing import List, Optional

import lcm
import numpy as np
import ompl.base as ob
import ompl.geometric as og
import open3d as o3d
import trimesh

from lcm_msgs.sensor_msgs import JointState, PointCloud2

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    DiagramBuilder,
    InverseKinematics,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Solve,
    StartMeshcat,
)
from pydrake.geometry import (
    CollisionFilterDeclaration,
    Mesh,
    ProximityProperties,
    InMemoryMesh,
    Box,
    Cylinder,
)
from pydrake.math import RigidTransform as DrakeRigidTransform
from pydrake.common import MemoryFile

from rgb_to_convex_sam2 import run_pipeline  # returns List[o3d.geometry.TriangleMesh]


class Planner:
    def __init__(self):
        # 1. Build Drake diagram, plant, and scene graph
        self.meshcat = StartMeshcat()
        self.urdf_path = os.path.abspath("../assets/devkit_base_descr.urdf")

        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.01)
        self.parser = Parser(self.plant)

        # Load the robot URDF
        self.model_instances = self.parser.AddModelsFromUrl(f"file://{self.urdf_path}")
        # Robot’s kinematic chain joints (7-DOF arm)
        self.kinematic_chain_joints = [
            "pillar_platform_joint",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.model_instance = self.model_instances[0] if self.model_instances else None

        # Exclude self-collisions on the arm links
        links_to_ignore = [
            "devkit_base_link",
            "pillar_platform",
            "piper_angled_mount",
            "pan_tilt_base",
            "pan_tilt_head",
            "pan_tilt_pan",
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
        ]
        bodies = []
        for link_name in links_to_ignore:
            body = self.plant.GetBodyByName(link_name)
            if body is not None:
                bodies.extend(self.plant.GetBodiesWeldedTo(body))

        arm_geoms = self.plant.CollectRegisteredGeometries(bodies)
        decl = CollisionFilterDeclaration().ExcludeWithin(arm_geoms)
        manager = self.scene_graph.collision_filter_manager()
        manager.Apply(decl)

        # 2. Add convex-hull meshes as collision/visual geometry
        convex_hulls = run_pipeline()
        self._register_convex_hulls_as_collision(convex_hulls)

        # 3. (Optional) LCM subscription for point clouds, etc.
        self.lc = lcm.LCM()
        self.latest_pointcloud = None
        self.pointcloud_lock = threading.Lock()
        # If you need to subscribe to pointclouds, uncomment:
        # self.lc.subscribe("head_cam_pointcloud#sensor_msgs.PointCloud2", self._handle_pointcloud_msg)
        
        self.lcm_thread = Thread(target=self._run_lcm, daemon=True)
        self.lcm_thread.start()

        # 4. Finalize the plant (no more geometry after this)
        self.plant.Finalize()

        # 7. Meshcat visualizer for the robot
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat, params=MeshcatVisualizerParams()
        )

        # 5. Build the diagram and contexts
        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

        # 6. Collect indices of the actuated joints for planning
        self.joint_indices = []
        for joint_name in self.kinematic_chain_joints:
            try:
                joint = self.plant.GetJointByName(joint_name)
                if joint.num_positions() > 0:
                    start_index = joint.position_start()
                    for i in range(joint.num_positions()):
                        self.joint_indices.append(start_index + i)
            except RuntimeError:
                print(f"[Warning] Joint '{joint_name}' not found in URDF.")

        # 8. End-effector references
        self.end_effector_link = self.plant.GetBodyByName("link6")
        self.end_effector_frame = self.plant.GetFrameByName("link6")
        self.camera_link = self.plant.GetBodyByName("camera_center_link")

        # 9. Setup OMPL state space
        self._setup_ompl_space()

    def _run_lcm(self):
        """Continuously handle LCM messages."""
        while True:
            self.lc.handle()

    def _handle_pointcloud_msg(self, channel: str, data: bytes) -> None:
        """Decode incoming PointCloud2 to (N,3) array."""
        try:
            msg = PointCloud2.decode(data)
            assert msg.point_step == 16, f"Unexpected point_step: {msg.point_step}"
            floats = struct.unpack(f"{len(msg.data) // 4}f", msg.data)
            arr = np.array(floats, dtype=np.float32).reshape(-1, 4)[:, :3]
            with self.pointcloud_lock:
                self.latest_pointcloud = arr
            print(f"[LCM] Received point cloud: {arr.shape[0]} points.")
        except Exception as e:
            print(f"[LCM] Error decoding point cloud: {e}")

    def _register_convex_hulls_as_collision(self, meshes: List[o3d.geometry.TriangleMesh]) -> None:
        """
        Given a list of Open3D TriangleMesh objects (convex hulls),
        convert each to trimesh.Trimesh, export to an in-memory .obj,
        then register it as collision + visual geometry in the world frame.
        """
        world = self.plant.world_body()
        proximity = ProximityProperties()

        for i, mesh in enumerate(meshes):
            # Convert Open3D → numpy arrays → trimesh.Trimesh
            vertices = np.asarray(mesh.vertices)
            faces = np.asarray(mesh.triangles)
            tmesh = trimesh.Trimesh(vertices=vertices, faces=faces)

            # Export to OBJ in memory
            tmesh_obj_blob = tmesh.export(file_type="obj")
            mem_file = MemoryFile(
                contents=tmesh_obj_blob,
                extension=".obj",
                filename_hint=f"convex_hull_{i}.obj"
            )
            in_memory_mesh = InMemoryMesh()
            in_memory_mesh.mesh_file = mem_file
            drake_mesh = Mesh(in_memory_mesh, scale=1.0)

            # # Assign poses for glass (i=0) and bottle (i=1)
            # if i == 0:
            #     pos = np.array([0.0, 0.0, 0.0])
            #     rpy = RollPitchYaw(0.0, 0.0, 0.0)
            # elif i == 1:
            #     pos = np.array([0.0, 0.0, 0.0])
            #     rpy = RollPitchYaw(0.0, 0.0, 0.0)
            # else:
            #     # If there are more objects, place them all at the same location
            pos = np.array([0.0, 0.0, 0.0])
            rpy = RollPitchYaw(0.0, 0.0, 0.0)

            X_WG = DrakeRigidTransform(RotationMatrix(rpy), pos)

            # Use X_WG directly as X_BG since body=world
            self.plant.RegisterCollisionGeometry(
                body=world,
                X_BG=X_WG,
                shape=drake_mesh,
                name=f"convex_hull_collision_{i}",
                properties=proximity,
            )
            self.plant.RegisterVisualGeometry(
                body=world,
                X_BG=X_WG,
                shape=drake_mesh,
                name=f"convex_hull_visual_{i}",
                diffuse_color=np.array([0.7, 0.7, 0.7, 1.0]),
            )

            print(f"[Main] Registered index {i} mesh at pos={pos}, rpy={rpy.vector()}")

        # Add a simple table so objects are not floating in space
        table_shape = Box(1.0, 1.0, 1.0)
        table_pose = RigidTransform(p=[-1.0, 0.0, 0.5])
        self.plant.RegisterCollisionGeometry(world, table_pose, table_shape, "table_collision", proximity)
        self.plant.RegisterVisualGeometry(world, table_pose, table_shape, "table_visual", [0.8, 0.8, 0.8, 1.0])

    def _setup_ompl_space(self) -> None:
        """Define OMPL real-vector state space and bounds from plant joint limits."""
        num_joints = len(self.joint_indices)
        self.ompl_space = ob.RealVectorStateSpace(num_joints)
        bounds = ob.RealVectorBounds(num_joints)
        lower = self.plant.GetPositionLowerLimits()[self.joint_indices]
        upper = self.plant.GetPositionUpperLimits()[self.joint_indices]
        for i in range(num_joints):
            bounds.setLow(i, lower[i])
            bounds.setHigh(i, upper[i])
        self.ompl_space.setBounds(bounds)
        self.ompl_ss = og.SimpleSetup(self.ompl_space)

    def inverse_kinematics_full(self, target_pose: RigidTransform) -> np.ndarray:
        ik = InverseKinematics(self.plant, self.plant_context)
        q_vars = ik.q()  # length = total # positions (e.g. 11)

        # 1) Position constraint (±5 mm)
        p_BQ = np.zeros((3, 1))
        p_target = target_pose.translation().reshape((3, 1))
        tol_pos = 0.005
        ik.AddPositionConstraint(
            frameB=self.end_effector_frame,
            p_BQ=p_BQ,
            frameA=self.plant.world_frame(),
            p_AQ_lower=(p_target - tol_pos),
            p_AQ_upper=(p_target + tol_pos),
        )

        # 2) (Optionally) Orientation constraint (±0.1 rad)
        # ik.AddOrientationConstraint(
        #     frameAbar=self.plant.world_frame(),
        #     R_AbarA=target_pose.rotation(),
        #     frameBbar=self.end_effector_frame,
        #     R_BbarB=RotationMatrix(),
        #     theta_bound=0.1,
        # )

        prog = ik.get_mutable_prog()

        # 3) Initial guess = current full configuration (length=11), so sizes match
        full_q = self.plant.GetPositions(self.plant_context)
        prog.SetInitialGuess(q_vars, full_q)

        result = Solve(prog)
        if not result.is_success():
            raise RuntimeError("Full-DOF IK failed.")
        else:
            print("[IK] Full-DOF IK succeeded.")

        q_sol_full = result.GetSolution(q_vars).flatten()  # shape = (11,)
        # Return only the 7 arm joints to feed into your planner
        return q_sol_full[self.joint_indices]  # (7,)

    def forward_kinematics(self, joint_positions: np.ndarray) -> DrakeRigidTransform:
        """
        Given a length-7 array of joint_positions (arm will),
        compute and return the world-frame pose of the end-effector.

        Steps:
          1) Save the current plant positions.
          2) Set the plant to `joint_positions` (fills in only those 7 entries;
             leaves the other DOFs unchanged).
          3) Evaluate the end-effector's world pose.
          4) (Optionally) Restore the original plant state so you do not disturb
             any other computations.
        """
        # 1) Save current full-DOF positions
        full_q_before = self.plant.GetPositions(self.plant_context).copy()

        # 2) Set only the 7 actuated joints
        full_q = full_q_before.copy()
        for i, idx in enumerate(self.joint_indices):
            full_q[idx] = joint_positions[i]
        self.plant.SetPositions(self.plant_context, full_q)

        # 3) Evaluate the end-effector pose in the world frame
        X_W_EE = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_link)

        # 4) Restore the original plant state (so that subsequent code isn't affected)
        self.plant.SetPositions(self.plant_context, full_q_before)

        return X_W_EE

    def plan(self, start_q: np.ndarray, goal_q: np.ndarray):
        """
        Solve a motion-planning problem from start_q → goal_q using OMPL’s RRT*.
        start_q and goal_q are length-n arrays, where n = len(self.joint_indices).
        """
        # 1. OMPL states
        start = ob.State(self.ompl_space)
        goal = ob.State(self.ompl_space)
        for i in range(len(self.joint_indices)):
            start[i] = start_q[i]
            goal[i] = goal_q[i]

        self.ompl_ss.setStartAndGoalStates(start, goal)

        # 2. State validity checker: check for collisions
        # def is_valid(state):
        #     q = [state[i] for i in range(len(self.planner.joint_indices))]
        #     self.set_joint_positions(q)
        #     query = self.scene_graph.get_query_output_port().Eval(
        #         self.scene_graph.GetMyContextFromRoot(self.diagram_context)
        #     )
        #     for sd in query.ComputeSignedDistancePairwiseClosestPoints(-1e-6):
        #         return False
        #     return True

        # self.ompl_ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_valid))

        si = self.ompl_ss.getSpaceInformation()
        # clearance_obj = ClearanceOptimizationObjective(si, planner_self=self, clearance_threshold=-0.01)
        # self.ompl_ss.setOptimizationObjective(clearance_obj)

        # 3. RRT* planner
        planner = og.RRTstar(si)
        self.ompl_ss.setPlanner(planner)

        print("[OMPL] Solving motion plan...")
        solved = self.ompl_ss.solve(5.0)
        if solved:
            print("[OMPL] Planning succeeded.")
            path = self.ompl_ss.getSolutionPath()
            path.interpolate(100)
            self.path = path
            return path
        else:
            print("[OMPL] Planning failed.")
            sys.exit(1)

    def set_joint_positions(self, positions: List[float]) -> None:
        """Set actuated joints to `positions` (length = len(self.joint_indices))."""
        full_q = self.plant.GetPositions(self.plant_context)
        for i, idx in enumerate(self.joint_indices):
            full_q[idx] = positions[i]
        self.plant.SetPositions(self.plant_context, full_q)

    def get_joint_positions(self) -> np.ndarray:
        """Return current actuated-joint positions as a numpy array."""
        full_q = self.plant.GetPositions(self.plant_context)
        return full_q[self.joint_indices]

    def get_end_effector_position(self) -> DrakeRigidTransform:
        """Return world frame pose of the end-effector link (uses current plant state)."""
        return self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_link)

    def execute(self, visualize_in_meshcat: bool = True) -> None:
        """Interpolate and replay the planned path in Meshcat, while publishing joint states over LCM."""
        if not hasattr(self, "path"):
            print("[Execute] No planned path to execute.")
            return

        count = self.path.getStateCount()
        print(f"[Execute] Path has {count} states.")

        # List of all joint names in the robot (in order of full plant)
        full_joint_names = [
            "pillar_platform_joint",
            "pan_tilt_pan_joint",
            "pan_tilt_head_joint",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            # Add any additional joints if your URDF has them
        ]

        for i in range(count):
            # 1) Extract the OMPL state into a length-n vector of actuated joints
            q_ompl = [self.path.getState(i)[j] for j in range(len(self.joint_indices))]

            # 2) Apply those to the plant
            self.set_joint_positions(q_ompl)

            # 3) Visualize in Meshcat
            if visualize_in_meshcat:
                self.diagram.ForcedPublish(self.diagram_context)

            # 4) Build a full-length joint-position array for the JointState message
            full_q = [0.0] * len(full_joint_names)
            for val, idx in zip(q_ompl, self.joint_indices):
                full_q[idx] = val

            # 5) Publish over LCM
            msg = JointState()
            msg.header.stamp.sec = int(time.time())
            msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
            msg.header.frame_id = "base_link"
            msg.name = full_joint_names
            msg.position = full_q
            msg.velocity = []
            msg.effort = []
            msg.name_length = len(msg.name)
            msg.position_length = len(msg.position)
            msg.velocity_length = 0
            msg.effort_length = 0
            self.lc.publish("joint_states#sensor_msgs.JointState", msg.encode())

            # 6) Small delay for smooth playback
            time.sleep(0.05)

    def move_gripper(
        self,
        open: Optional[bool] = None,
        close: Optional[bool] = None,
        gripper_position: Optional[float] = None
    ) -> None:
        """
        Move the two gripper fingers symmetrically about the wrist.
        - If open=True, sets gripper_position = 0.035 m (fully open).
        - If close=True, sets gripper_position = 0.0 m (fully closed).
        - Otherwise, uses whatever gripper_position you passed in (must be within [0.0, 0.035]).
        """

        # 1) Determine the desired finger opening
        if open and close:
            raise ValueError("Cannot request both open and close at once.")
        if open:
            gripper_position = 0.035
        elif close:
            gripper_position = 0.0
        # else: assume caller provided a valid gripper_position

        # 2) Check the range
        if abs(gripper_position) > 0.035:
            raise ValueError("Gripper position must be between 0.0 and 0.035 m.")

        print(f"[Gripper] Moving gripper to {'open' if open else ('closed' if close else 'custom')} position {gripper_position:.3f} m.")

        # 3) Read the full plant positions (length = total # of joints)
        plant_positions = self.plant.GetPositions(self.plant_context)  # e.g. length 11

        # 4) Suppose index 9 is the left-finger joint, index 10 is the right-finger joint;
        #    set those two entries to ±gripper_position.
        #    (Adjust indices if your URDF uses different joint ordering.)
        if open or close or (gripper_position is not None):
            # Here, we assume that the gripper joints are exactly indices 9 and 10
            # in the “full” position vector. If that’s not correct, replace 9 and 10
            # with the proper joint indices for your gripper fingers.
            plant_positions[9] = gripper_position
            plant_positions[10] = -gripper_position

            self.plant.SetPositions(self.plant_context, plant_positions)
            # Optionally force a publish so Meshcat updates immediately:
            self.diagram.ForcedPublish(self.diagram_context)

        else:
            print("[Gripper] No open/close command or custom position provided, doing nothing.")


class ClearanceOptimizationObjective(ob.OptimizationObjective):
    def __init__(self, si, planner_self, clearance_threshold=0.0):
        """
        Args:
          si:                OMPL SpaceInformation object
          planner_self:      reference to your Planner instance (so we can query Drake)
          clearance_threshold: any signed-distance ≤ clearance_threshold is considered poor clearance.
        """
        super().__init__(si)
        self.planner = planner_self
        self.clearance_threshold = clearance_threshold

    def stateCost(self, state) -> ob.Cost:
        # 1) Extract a joint vector from the OMPL state (length = len(self.planner.joint_indices))
        q = [state[i] for i in range(len(self.planner.joint_indices))]

        # 2) Apply those joints to the Drake plant
        self.planner.set_joint_positions(q)
        self.planner.diagram.ForcedPublish(self.planner.diagram_context)

        # 3) Query Drake’s signed-distances
        query = self.planner.scene_graph.get_query_output_port().Eval(
            self.planner.scene_graph.GetMyContextFromRoot(self.planner.diagram_context)
        )
        signed_dists = [sd.distance for sd in query.ComputeSignedDistancePairwiseClosestPoints(self.clearance_threshold)]
        if not signed_dists:
            # No pairs returned implies robot is completely clear of any obstacle (all distances > threshold).
            # Assign a very low cost (best possible).
            return ob.Cost(0.0)

        # 4) Find the minimum signed distance among all pairs
        d_min = min(signed_dists)  # guaranteed > clearance_threshold, because collisions were filtered out separately

        # 5) Convert clearance to cost. For instance, cost = 1 / (d_min + ε)
        #    so states with larger clearance (big d_min) get smaller cost.
        eps = 1e-3  # To avoid division by zero
        cost_value = 1.0 / (d_min + eps)
        return ob.Cost(cost_value)

    def motionCost(self, s1, s2) -> ob.Cost:
        # By default, sum of state-costs along the motion
        return self.combineCosts(self.stateCost(s1), self.stateCost(s2))


def main():
    parser = argparse.ArgumentParser(description="Drake Planner with IK + Convex Hull Collision + FK")
    args = parser.parse_args()

    planner = Planner()

    # 1) Print the current end-effector pose (using FK on the current joint positions)
    # planner.set_joint_positions([-0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # planner.plant.SetPositions(planner.plant_context, [0.0, -0.5, -0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    current_joint_positions = planner.get_joint_positions()
    # X_W_EE_before = planner.forward_kinematics(current_joint_positions)
    X_W_EE_before = planner.plant.EvalBodyPoseInWorld(planner.plant_context, planner.end_effector_link)

    print(f"[Main] Current joint angles: {current_joint_positions}")
    print(f"[Main] Forward Kinematics (current EE pose):")
    print(f"         Translation = {X_W_EE_before.translation()}")
    quat_before = X_W_EE_before.rotation().ToQuaternion()
    print(f"         Rotation (w, x, y, z) = "
        f"[{quat_before.w():.6f}, {quat_before.x():.6f}, "
        f"{quat_before.y():.6f}, {quat_before.z():.6f}]")
    

    # 2) OPTIONAL: Use full-DOF IK to compute a feasible 7-DOF arm goal
    # target_pose = RigidTransform(
    #     RotationMatrix(RollPitchYaw(0.0, 0.0, 0.0)),
    #     np.array([-0.4, 0.2, 1.5])
    # )
    # try:
    #     goal_q = planner.inverse_kinematics_full(target_pose)
    # except RuntimeError as e:
    #     print(f"[IK] Failed to find solution: {e}")
    #     sys.exit(1)

    # 3) If you already have some desired goal configuration in joint space:
    # start_q = planner.get_joint_positions()
    # goal_q = np.array([-0.56, -0.02, 1.1, -0.7, 1.1, -0.8, -0.09])
    # print(f"[Main] Start config: {start_q}")
    # print(f"[Main] Goal config (assumed/precomputed): {goal_q}")
    # print(f"[Main] Upper bounds: {planner.plant.GetPositionUpperLimits()[planner.joint_indices]}")
    # print(f"[Main] Lower bounds: {planner.plant.GetPositionLowerLimits()[planner.joint_indices]}")
    print(f"[Main] Full DOF upper bounds: {planner.plant.GetPositionUpperLimits()}")
    print(f"[Main] Full DOF lower bounds: {planner.plant.GetPositionLowerLimits()}")

    # 4) Plan from start_q → goal_q
    # path = planner.plan(start_q, goal_q)

    # 5) After planning, print the end-effector pose at the goal configuration:
    # X_W_EE_goal = planner.forward_kinematics(goal_q)
    # print(f"[Main] Forward Kinematics (goal EE pose):")
    # print(f"         Translation = {X_W_EE_goal.translation()}")
    # print(f"         Rotation (quaternion) = {X_W_EE_goal.rotation().ToQuaternion().coeffs()}")

    # 6) Execute trajectory in Meshcat
    # planner.execute(visualize_in_meshcat=True)

    # 7) Fully open:
    planner.move_gripper(open=True)

    # 8) Fully closed:
    planner.move_gripper(close=True)

    # 9) Partially open (say 0.02 m between fingers):
    planner.move_gripper(gripper_position=0.02)

    # 10) Keep the script alive so Meshcat & LCM stay active
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Main] Exiting gracefully.")


if __name__ == "__main__":
    main()

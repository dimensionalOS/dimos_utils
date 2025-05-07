import numpy as np
import open3d as o3d  # For point cloud processing
import ompl.base as ob
import ompl.geometric as og
import ompl.util as ou
import pybullet as p  # For collision checking and forward kinematics
import pybullet_data
import time
import math
from scipy.spatial import KDTree  # For efficient point cloud distance queries

class MotionPlanner:
    def __init__(self, urdf_path, joint_names, point_cloud=None, collision_free=False):
        """
        Initialize the motion planner.
        
        Args:
            urdf_path: Path to the URDF file
            joint_names: List of joint names to use in planning
            point_cloud: Open3D point cloud object (optional)
            collision_free: If True, require completely collision-free paths
                           If False, minimize collisions with the point cloud
        """
        # Initialize PyBullet
        self.client_id = p.connect(p.DIRECT)  # Headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load the robot from URDF
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        
        # Get joint information
        self.joint_indices = []
        self.joint_limits = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name in joint_names:
                self.joint_indices.append(i)
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                self.joint_limits.append((lower_limit, upper_limit))
        
        # Store parameters
        self.point_cloud = point_cloud
        self.collision_free = collision_free
        self.point_tree = None
        
        # Process point cloud if provided
        if self.point_cloud is not None:
            self.process_point_cloud(self.point_cloud)
        
        # OMPL setup
        self.setup_ompl_space()
    
    def process_point_cloud(self, point_cloud):
        """Process the point cloud and prepare for collision checking."""
        # Convert to numpy array for faster processing
        self.points = np.asarray(point_cloud.points)
        
        # Create KD-Tree for efficient nearest neighbor queries
        self.point_tree = KDTree(self.points)
    
    def setup_ompl_space(self):
        """Set up the OMPL state space and problem definition."""
        # Create state space (one dimension per joint)
        self.space = ob.RealVectorStateSpace(len(self.joint_indices))
        
        # Set joint limits
        bounds = ob.RealVectorBounds(len(self.joint_indices))
        for i, (lower, upper) in enumerate(self.joint_limits):
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)
        self.space.setBounds(bounds)
        
        # Create simple setup
        self.ss = og.SimpleSetup(self.space)
        
        # Set state validity checker based on mode
        if self.collision_free:
            # In collision-free mode, use binary validity checker
            self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.binary_validity_checker))
            # Default planner will be set in the plan method
        else:
            # In collision-minimization mode, all states are technically "valid"
            self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.cost_validity_checker))
            # Set optimization objective for collision minimization
            objective = self.create_objective()
            self.ss.setOptimizationObjective(objective)
            # Default planner will be set in the plan method based on the task
        
        # We'll set the planner in the plan method to allow flexibility
    
    def binary_validity_checker(self, state):
        """
        Standard validity checker for collision-free planning.
        Returns True if state is collision-free, False otherwise.
        """
        # Apply joint positions
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, state[i])
        
        # Check for collisions with PyBullet
        # This uses PyBullet's built-in collision detection with the loaded environment
        # Could be extended to check against point cloud if desired
        for link_idx in range(p.getNumJoints(self.robot_id)):
            contact_points = p.getContactPoints(bodyA=self.robot_id, linkIndexA=link_idx)
            if len(contact_points) > 0:
                return False  # Collision detected
        
        return True  # No collisions
    
    def cost_validity_checker(self, state):
        """
        Custom validity checker that allows collisions but with a cost.
        
        Returns True for all states, but assigns cost through motion cost.
        """
        # Always return valid, costs are handled in the optimization objective
        return True
    
    def create_objective(self):
        """Create an optimization objective that minimizes collisions."""
        # Create a custom optimization objective
        obj = ob.OptimizationObjective(self.ss.getSpaceInformation())
        
        # Set motion cost function that evaluates collision with point cloud
        obj.setCostToGoHeuristic(ob.CostToGoHeuristic(self.cost_to_go))
        obj.setMotionCostFn(ob.MotionCostFn(self.motion_cost))
        
        return obj
    
    def cost_to_go(self, state):
        """Estimated cost to go to goal from this state."""
        # For now, just return a simple Euclidean distance to goal
        # This could be improved with a more sophisticated heuristic
        goal = self.ss.getGoal().getState()
        return ob.Cost(self.state_distance(state, goal))
    
    def state_distance(self, state1, state2):
        """Compute Euclidean distance between states."""
        dist = 0.0
        for i in range(len(self.joint_indices)):
            dist += (state1[i] - state2[i]) ** 2
        return math.sqrt(dist)
    
    def motion_cost(self, state1, state2):
        """
        Compute the cost of motion between two states.
        Incorporates the collision cost with the point cloud if available.
        """
        # Get joint configurations
        joints1 = [state1[i] for i in range(len(self.joint_indices))]
        joints2 = [state2[i] for i in range(len(self.joint_indices))]
        
        # Compute collision cost along the path
        # Interpolate between states for better collision checking
        num_steps = 10
        total_cost = 0.0
        
        for step in range(num_steps + 1):
            t = float(step) / num_steps
            interp_joints = [joints1[i] + t * (joints2[i] - joints1[i]) for i in range(len(joints1))]
            
            # Apply joint positions
            for joint_idx, joint_val in zip(self.joint_indices, interp_joints):
                p.resetJointState(self.robot_id, joint_idx, joint_val)
            
            # Check for collisions
            if self.point_cloud is not None:
                # If point cloud is available, compute cost based on it
                collision_cost = self.compute_point_cloud_collision_cost()
            else:
                # If no point cloud, use standard collision checking
                # Assign high cost to any collisions
                collision_detected = False
                for link_idx in range(p.getNumJoints(self.robot_id)):
                    contact_points = p.getContactPoints(bodyA=self.robot_id, linkIndexA=link_idx)
                    if len(contact_points) > 0:
                        collision_detected = True
                        break
                
                collision_cost = 100.0 if collision_detected else 0.0
            
            total_cost += collision_cost
        
        # Average cost along the path
        avg_cost = total_cost / (num_steps + 1)
        
        # Add distance component to encourage shorter paths
        distance_cost = self.state_distance(state1, state2)
        
        # Weighted sum of collision cost and distance cost
        # Adjust weights based on your priorities
        collision_weight = 0.8
        distance_weight = 0.2
        final_cost = collision_weight * avg_cost + distance_weight * distance_cost
        
        return ob.Cost(final_cost)
    
    def compute_point_cloud_collision_cost(self):
        """
        Compute how much the current robot configuration collides with the point cloud.
        Returns a cost value based on collision severity.
        """
        if self.point_tree is None:
            return 0.0  # No point cloud, no collision cost
            
        total_cost = 0.0
        
        # Get all link positions and check distances to point cloud
        for link_idx in range(p.getNumJoints(self.robot_id)):
            link_state = p.getLinkState(self.robot_id, link_idx)
            link_position = link_state[0]  # (x, y, z)
            
            # Query KD-Tree for nearest points in the point cloud
            distances, _ = self.point_tree.query(link_position, k=5)
            
            # Compute cost based on distances
            # Points that are very close contribute more to the cost
            for dist in distances:
                if dist < 0.1:  # Collision threshold
                    # Inverse relationship: closer points = higher cost
                    total_cost += 0.1 / (dist + 0.01)  # Avoid division by zero
        
        return total_cost
    
    def plan(self, start_joints, goal_link_name_or_idx, goal_position, goal_orientation, planning_time=5.0, max_attempts=3):
        """
        Plan a motion that minimizes collisions with the point cloud.
        
        Args:
            start_joints: Starting joint positions
            goal_link_name_or_idx: Name or index of the link to reach the goal
            goal_position: Target position for the link
            goal_orientation: Target orientation for the link (quaternion)
            planning_time: Time budget for planning in seconds
            max_attempts: Maximum number of planning attempts to find a solution that meets constraints
            
        Returns:
            List of joint configurations along the path
        """
        # Determine if goal_link_name_or_idx is an index or a name
        goal_link_idx = None
        if isinstance(goal_link_name_or_idx, int):
            goal_link_idx = goal_link_name_or_idx
            if goal_link_idx >= p.getNumJoints(self.robot_id):
                raise ValueError(f"Goal link index {goal_link_idx} is out of range")
        else:
            # It's a name, need to find the index
            for i in range(p.getNumJoints(self.robot_id)):
                joint_info = p.getJointInfo(self.robot_id, i)
                link_name = joint_info[12].decode('utf-8')
                joint_name = joint_info[1].decode('utf-8')
                if link_name == goal_link_name_or_idx or joint_name == goal_link_name_or_idx:
                    goal_link_idx = i
                    break
            
            if goal_link_idx is None:
                raise ValueError(f"Goal link '{goal_link_name_or_idx}' not found in URDF")
        
        # Define base thresholds for position and orientation
        pos_threshold = 0.05      # 5cm for position
        orient_threshold = 0.1    # Enforce orientation error below 0.1
        
        # Set up planner based on collision mode
        si = self.ss.getSpaceInformation()
        if self.collision_free:
            # For collision-free planning, RRTConnect is efficient
            planner = og.RRTConnect(si)
        else:
            # For collision minimization, RRT* is better as it optimizes paths
            planner = og.RRTstar(si)
            # Configure RRT* for better optimization
            planner.setRange(0.1)  # Step size
            planner.setSolutionHeuristic(True)  # Use heuristic to improve solutions
        
        # Multiple planning attempts to ensure orientation threshold is met
        best_path = None
        best_orient_error = float('inf')
        best_pos_error = float('inf')
        
        for attempt in range(max_attempts):
            print(f"\nPlanning attempt {attempt+1}/{max_attempts}")
            
            # Set start state
            start = ob.State(self.space)
            for i, joint_val in enumerate(start_joints):
                start[i] = joint_val
            self.ss.setStartState(start)
            
            # With each attempt, use increasingly aggressive sampling techniques
            valid_goal_configs = []
            
            # Progressive constraint relaxation with each attempt
            curr_pos_threshold = pos_threshold * (1 + 0.5 * attempt)
            curr_orient_threshold = orient_threshold * (1 + 0.5 * attempt)
            
            # 1. First try PyBullet's builtin IK to get some initial goal candidates
            valid_goal_configs.extend(self._sample_with_pybullet_ik(
                goal_link_idx, goal_position, goal_orientation, 
                curr_pos_threshold, curr_orient_threshold))
            
            # 2. If we still need more, try random sampling with different strategies
            if len(valid_goal_configs) < 5:
                valid_goal_configs.extend(self._sample_random_goals(
                    goal_link_idx, goal_position, goal_orientation,
                    curr_pos_threshold, curr_orient_threshold, attempt))
            
            # 3. If we still need more, try a grid-based approach on likely joints
            if len(valid_goal_configs) < 5:
                valid_goal_configs.extend(self._sample_grid_goals(
                    goal_link_idx, goal_position, goal_orientation,
                    curr_pos_threshold, curr_orient_threshold, attempt))
                
            # 4. As a last resort, use goal biasing to find configurations
            if len(valid_goal_configs) < 1:
                valid_goal_configs.extend(self._sample_with_goal_biasing(
                    goal_link_idx, goal_position, goal_orientation))
            
            print(f"Found {len(valid_goal_configs)} configurations that satisfy constraints")
            
            # If we still don't have any configurations, this is very unusual
            # In this case, just use the best we found from all our sampling methods
            if len(valid_goal_configs) == 0:
                print("No valid configurations found, using closest approximations")
                valid_goal_configs = self._get_closest_approximations(
                    goal_link_idx, goal_position, goal_orientation)
            
            # Create multi-goal objective with the configurations we found
            multi_goal = ob.GoalStates(si)
            for config in valid_goal_configs[:10]:  # Use up to 10 goals
                goal_state = ob.State(self.space)
                for i, val in enumerate(config):
                    goal_state[i] = val
                multi_goal.addState(goal_state)
            
            # Set the goal in our planner
            self.ss.setGoal(multi_goal)
            self.ss.setPlanner(planner)
            
            # Solve
            solved = self.ss.solve(planning_time)
            
            if not solved:
                print(f"Attempt {attempt+1}: No solution found")
                continue
                
            # Extract and evaluate path
            path = self.ss.getSolutionPath()
            path.interpolate(100)
            
            # Extract joint values
            joint_trajectory = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                joints = [state[j] for j in range(len(self.joint_indices))]
                joint_trajectory.append(joints)
            
            # Evaluate final orientation error
            final_joints = joint_trajectory[-1]
            for i, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, final_joints[i])
            
            final_state = p.getLinkState(self.robot_id, goal_link_idx)
            final_pos = final_state[0]
            final_orn = final_state[1]
            
            # Calculate errors
            pos_error = math.sqrt(sum((final_pos[i] - goal_position[i])**2 for i in range(3)))
            dot = sum(final_orn[i] * goal_orientation[i] for i in range(4))
            dot = max(-1.0, min(dot, 1.0))
            orient_error = 1.0 - dot**2
            
            print(f"Attempt {attempt+1} results:")
            print(f"  Position error: {pos_error:.4f}m")
            print(f"  Orientation error: {orient_error:.4f}")
            
            # Keep the best path so far
            if orient_error < best_orient_error or (orient_error <= orient_threshold and pos_error < best_pos_error):
                best_path = joint_trajectory
                best_orient_error = orient_error
                best_pos_error = pos_error
                
                # If we've found a solution that meets the threshold, stop early
                if orient_error <= orient_threshold:
                    print(f"Found solution meeting orientation threshold on attempt {attempt+1}")
                    break
        
        # If we found any solution, return the best one
        if best_path:
            print("\nFinal solution:")
            print(f"  Position error: {best_pos_error:.4f}m")
            print(f"  Orientation error: {best_orient_error:.4f}")
            
            if best_orient_error <= orient_threshold:
                print(f"Success! Orientation error ({best_orient_error:.4f}) is below threshold ({orient_threshold:.4f})")
            else:
                print(f"Warning: Best orientation error ({best_orient_error:.4f}) exceeds threshold ({orient_threshold:.4f})")
                # Try to improve the solution with local optimization if needed
                if best_orient_error < 0.15:  # Not too far from threshold
                    improved_path = self._refine_orientation(best_path, goal_link_idx, goal_orientation, orient_threshold)
                    if improved_path:
                        best_path = improved_path
                        print("Successfully refined path to meet orientation threshold!")
            
            return best_path
        else:
            print("No solution found after all attempts - this should never happen!")
            return None

    def _sample_with_pybullet_ik(self, goal_link_idx, goal_position, goal_orientation, pos_threshold, orient_threshold):
        """Try to find configurations using PyBullet's IK solver with error handling"""
        print("Sampling with PyBullet IK...")
        valid_configs = []
        
        try:
            # Use PyBullet's IK solver to get candidate configurations
            for _ in range(20):  # Try multiple initial guesses for IK
                # Create a random initial guess
                initial_guess = []
                for lower, upper in self.joint_limits:
                    initial_guess.append(lower + np.random.random() * (upper - lower))
                
                # Call PyBullet's IK solver
                # Use null space parameters to handle redundant DoF better
                # Lower and upper limits for the null space search
                ll = [lower for lower, _ in self.joint_limits]
                ul = [upper for _, upper in self.joint_limits]
                
                # Joint ranges for null space optimization
                jr = [upper - lower for lower, upper in self.joint_limits]
                
                # Add some noise to the rest positions to improve exploration
                rp = [0.5 * (lower + upper) + 0.1 * (np.random.random() - 0.5) * (upper - lower) 
                    for lower, upper in self.joint_limits]
                
                try:
                    # Call the IK solver with null space parameters
                    ik_solution = p.calculateInverseKinematics(
                        self.robot_id, 
                        goal_link_idx, 
                        goal_position, 
                        goal_orientation,
                        lowerLimits=ll,
                        upperLimits=ul,
                        jointRanges=jr,
                        restPoses=rp,
                        maxNumIterations=100,
                        residualThreshold=1e-4
                    )
                    
                    # Safely extract only the joints we're using for planning
                    solution = []
                    for idx in self.joint_indices:
                        if idx < len(ik_solution):
                            solution.append(ik_solution[idx])
                        else:
                            # If index out of range, use initial guess for that joint
                            solution.append(initial_guess[self.joint_indices.index(idx)])
                    
                    # Only proceed if we have a solution for all joints
                    if len(solution) == len(self.joint_indices):
                        # Check if the solution is valid
                        for i, joint_idx in enumerate(self.joint_indices):
                            p.resetJointState(self.robot_id, joint_idx, solution[i])
                        
                        # Check the resulting pose
                        link_state = p.getLinkState(self.robot_id, goal_link_idx)
                        pos = link_state[0]
                        orn = link_state[1]
                        
                        # Calculate errors
                        pos_error = math.sqrt(sum((pos[i] - goal_position[i])**2 for i in range(3)))
                        dot = sum(orn[i] * goal_orientation[i] for i in range(4))
                        dot = max(-1.0, min(dot, 1.0))
                        orient_error = 1.0 - dot**2
                        
                        # If it's a valid solution, add it
                        if pos_error < pos_threshold and orient_error < orient_threshold:
                            valid_configs.append(solution)
                            if len(valid_configs) >= 5:  # If we have enough, stop sampling
                                break
                except Exception as e:
                    print(f"IK calculation error: {e}")
                    continue  # Try next iteration if IK fails
        
        except Exception as e:
            print(f"PyBullet IK sampling failed: {e}")
        
        print(f"Found {len(valid_configs)} configurations from IK")
        return valid_configs

    def _sample_random_goals(self, goal_link_idx, goal_position, goal_orientation, pos_threshold, orient_threshold, attempt_number):
        """Sample random configurations with smarter strategies"""
        print("Sampling with randomized strategies...")
        valid_configs = []
        
        # Number of samples increases with each attempt
        num_samples = 2000 * (attempt_number + 1)
        
        # Track all sampled configurations and their errors for fallback
        all_configs = []
        all_pos_errors = []
        all_orient_errors = []
        
        for i in range(num_samples):
            if i % 500 == 0:
                print(f"  Progress: {i}/{num_samples}")
            
            # Use different sampling strategies
            config = [0.0] * len(self.joint_indices)
            
            # Sampling strategy varies based on attempt number
            if attempt_number == 0:
                # Simple random sampling
                for j, (lower, upper) in enumerate(self.joint_limits):
                    config[j] = lower + np.random.random() * (upper - lower)
            else:
                # More sophisticated sampling that focuses on promising regions
                for j, (lower, upper) in enumerate(self.joint_limits):
                    # Use different distributions for different joints
                    if j < 3:  # First few joints control major arm positioning
                        # Use a biased distribution to focus more samples in useful areas
                        t = np.random.beta(2, 2)  # Beta distribution concentrates values in middle
                        config[j] = lower + t * (upper - lower)
                    else:
                        # For wrist joints, use uniform sampling
                        config[j] = lower + np.random.random() * (upper - lower)
            
            # Set the robot to this configuration
            for j, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, config[j])
            
            # Check the resulting pose
            link_state = p.getLinkState(self.robot_id, goal_link_idx)
            pos = link_state[0]
            orn = link_state[1]
            
            # Calculate errors
            pos_error = math.sqrt(sum((pos[i] - goal_position[i])**2 for i in range(3)))
            dot = sum(orn[i] * goal_orientation[i] for i in range(4))
            dot = max(-1.0, min(dot, 1.0))
            orient_error = 1.0 - dot**2
            
            # Track all configurations for fallback
            all_configs.append(config)
            all_pos_errors.append(pos_error)
            all_orient_errors.append(orient_error)
            
            # If it meets our criteria, add to valid configs
            if pos_error < pos_threshold and orient_error < orient_threshold:
                valid_configs.append(config)
                if len(valid_configs) >= 10:  # If we have enough, stop sampling
                    break
        
        # If we didn't find enough configurations, use the best ones we found
        if len(valid_configs) < 5 and all_configs:
            # Compute combined score prioritizing orientation
            scores = [2*orient_err + pos_err for orient_err, pos_err in zip(all_orient_errors, all_pos_errors)]
            best_indices = np.argsort(scores)[:5]
            
            for idx in best_indices:
                if all_configs[idx] not in valid_configs:
                    valid_configs.append(all_configs[idx])
        
        print(f"Found {len(valid_configs)} configurations from random sampling")
        return valid_configs

    def _sample_grid_goals(self, goal_link_idx, goal_position, goal_orientation, pos_threshold, orient_threshold, attempt_number):
        """Sample using a grid-based approach on key joints"""
        print("Sampling with grid-based approach...")
        valid_configs = []
        
        # Use grid sampling for the first 3-4 joints, which typically have the most impact
        grid_size = 5 + attempt_number  # Increase grid resolution with each attempt
        
        # For 6-7 DOF arms, the first 3-4 joints have the most impact on end effector pose
        num_grid_joints = min(4, len(self.joint_indices))
        
        # Generate grid points for each joint
        grid_points = []
        for i in range(num_grid_joints):
            lower, upper = self.joint_limits[i]
            points = np.linspace(lower, upper, grid_size)
            grid_points.append(points)
        
        # For remaining joints, use fixed values or random sampling
        remaining_joints = []
        for i in range(num_grid_joints, len(self.joint_indices)):
            lower, upper = self.joint_limits[i]
            # Use a few values for remaining joints
            points = np.linspace(lower, upper, 3)
            remaining_joints.append(points)
        
        # Track all configurations and their errors
        all_configs = []
        all_pos_errors = []
        all_orient_errors = []
        
        # Generate combinations - start with fewer to avoid combinatorial explosion
        max_combinations = 500
        combinations_generated = 0
        
        # Get all combinations for grid joints
        import itertools
        grid_combinations = list(itertools.product(*grid_points))
        
        # Shuffle to get a random subset if there are too many
        np.random.shuffle(grid_combinations)
        
        for grid_combo in grid_combinations:
            # Stop if we've generated enough combinations
            if combinations_generated >= max_combinations:
                break
                
            # For each grid combination, try a few combinations of remaining joints
            for _ in range(3):  # Try 3 random combinations for remaining joints
                config = list(grid_combo)
                
                # Add values for remaining joints
                for i, points in enumerate(remaining_joints):
                    idx = i + num_grid_joints
                    # Pick a random value from the options
                    config.append(np.random.choice(points))
                
                combinations_generated += 1
                
                # Set the robot to this configuration
                for j, joint_idx in enumerate(self.joint_indices):
                    p.resetJointState(self.robot_id, joint_idx, config[j])
                
                # Check the resulting pose
                link_state = p.getLinkState(self.robot_id, goal_link_idx)
                pos = link_state[0]
                orn = link_state[1]
                
                # Calculate errors
                pos_error = math.sqrt(sum((pos[k] - goal_position[k])**2 for k in range(3)))
                dot = sum(orn[k] * goal_orientation[k] for k in range(4))
                dot = max(-1.0, min(dot, 1.0))
                orient_error = 1.0 - dot**2
                
                # Track all configurations
                all_configs.append(config)
                all_pos_errors.append(pos_error)
                all_orient_errors.append(orient_error)
                
                # If it meets criteria, add to valid configs
                if pos_error < pos_threshold and orient_error < orient_threshold:
                    valid_configs.append(config)
                    if len(valid_configs) >= 10:
                        break
            
            if len(valid_configs) >= 10:
                break
        
        # If we didn't find enough configurations, use the best ones we found
        if len(valid_configs) < 5 and all_configs:
            # Compute combined score (weighted sum of errors)
            scores = [2*orient_err + pos_err for orient_err, pos_err in zip(all_orient_errors, all_pos_errors)]
            best_indices = np.argsort(scores)[:min(5, len(scores))]
            
            for idx in best_indices:
                if all_configs[idx] not in valid_configs:
                    valid_configs.append(all_configs[idx])
        
        print(f"Found {len(valid_configs)} configurations from grid sampling")
        return valid_configs

    def _sample_with_goal_biasing(self, goal_link_idx, goal_position, goal_orientation):
        """Use goal-biased sampling as a last resort"""
        print("Using goal-biased sampling as fallback...")
        valid_configs = []
        
        # Start from current configuration
        current_config = []
        for joint_idx in self.joint_indices:
            joint_state = p.getJointState(self.robot_id, joint_idx)
            current_config.append(joint_state[0])
        
        # Perform goal-biased random walk
        best_config = current_config
        best_pos_error = float('inf')
        best_orient_error = float('inf')
        
        # Try many small random steps
        for _ in range(1000):
            # Take a small random step
            test_config = best_config.copy()
            
            # Random perturbation
            for i in range(len(test_config)):
                step_size = (self.joint_limits[i][1] - self.joint_limits[i][0]) * 0.05
                test_config[i] += np.random.uniform(-step_size, step_size)
                # Clamp to joint limits
                test_config[i] = max(self.joint_limits[i][0], min(self.joint_limits[i][1], test_config[i]))
            
            # Set the robot to this configuration
            for i, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, test_config[i])
            
            # Check the resulting pose
            link_state = p.getLinkState(self.robot_id, goal_link_idx)
            pos = link_state[0]
            orn = link_state[1]
            
            # Calculate errors
            pos_error = math.sqrt(sum((pos[i] - goal_position[i])**2 for i in range(3)))
            dot = sum(orn[i] * goal_orientation[i] for i in range(4))
            dot = max(-1.0, min(dot, 1.0))
            orient_error = 1.0 - dot**2
            
            # Keep the best configuration so far
            # Prioritize improving the most problematic error
            current_worst = max(best_pos_error / 0.1, best_orient_error / 0.1)
            new_worst = max(pos_error / 0.1, orient_error / 0.1)
            
            if new_worst < current_worst:
                best_config = test_config
                best_pos_error = pos_error
                best_orient_error = orient_error
        
        # Add the best configuration we found
        valid_configs.append(best_config)
        print(f"Goal-biased sampling found 1 configuration (errors: pos={best_pos_error:.4f}, orient={best_orient_error:.4f})")
        
        return valid_configs

    def _get_closest_approximations(self, goal_link_idx, goal_position, goal_orientation):
        """Last resort: find configurations that are as close as possible to the goal"""
        print("Finding best approximation to goal...")
        
        # This should only happen if all previous methods failed
        all_configs = []
        all_errors = []
        
        # Try a large number of random configurations
        for _ in range(5000):
            config = [0.0] * len(self.joint_indices)
            for i, (lower, upper) in enumerate(self.joint_limits):
                config[i] = lower + np.random.random() * (upper - lower)
            
            # Set the robot to this configuration
            for i, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, config[i])
            
            # Check the resulting pose
            link_state = p.getLinkState(self.robot_id, goal_link_idx)
            pos = link_state[0]
            orn = link_state[1]
            
            # Calculate errors
            pos_error = math.sqrt(sum((pos[i] - goal_position[i])**2 for i in range(3)))
            dot = sum(orn[i] * goal_orientation[i] for i in range(4))
            dot = max(-1.0, min(dot, 1.0))
            orient_error = 1.0 - dot**2
            
            # Combined error with emphasis on orientation
            combined_error = 2 * orient_error + pos_error
            
            all_configs.append(config)
            all_errors.append(combined_error)
        
        # Sort by error and take the best ones
        best_indices = np.argsort(all_errors)[:5]
        best_configs = [all_configs[i] for i in best_indices]
        
        print(f"Found 5 closest approximations to goal")
        return best_configs

    def _refine_orientation(self, path, goal_link_idx, goal_orientation, orient_threshold):
        """Try to refine the final pose to improve orientation error"""
        print("Attempting local refinement to improve orientation...")
        
        # Get the final configuration
        final_config = path[-1]
        
        # Setup for local optimization
        best_config = final_config.copy()
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, best_config[i])
        
        final_state = p.getLinkState(self.robot_id, goal_link_idx)
        final_orn = final_state[1]
        
        # Calculate initial orientation error
        dot = sum(final_orn[i] * goal_orientation[i] for i in range(4))
        dot = max(-1.0, min(dot, 1.0))
        best_error = 1.0 - dot**2
        
        # Simple local search to improve orientation
        step_sizes = [0.05, 0.02, 0.01, 0.005]  # Multiple step sizes for multi-resolution search
        max_iterations = 50
        
        for step_size in step_sizes:
            improved = True
            iteration = 0
            
            while improved and iteration < max_iterations and best_error > orient_threshold:
                improved = False
                iteration += 1
                
                # Try small adjustments to each joint
                for i in range(len(self.joint_indices)):
                    # Skip first joint if it's just controlling height
                    if i == 0 and len(self.joint_indices) > 3:
                        continue
                    
                    for direction in [-1, 1]:
                        # Try changing this joint
                        test_config = best_config.copy()
                        test_config[i] += direction * step_size
                        
                        # Check joint limits
                        lower, upper = self.joint_limits[i]
                        if test_config[i] < lower or test_config[i] > upper:
                            continue
                        
                        # Apply and test
                        for j, joint_idx in enumerate(self.joint_indices):
                            p.resetJointState(self.robot_id, joint_idx, test_config[j])
                        
                        test_state = p.getLinkState(self.robot_id, goal_link_idx)
                        test_orn = test_state[1]
                        
                        # Calculate orientation error
                        dot = sum(test_orn[j] * goal_orientation[j] for j in range(4))
                        dot = max(-1.0, min(dot, 1.0))
                        test_error = 1.0 - dot**2
                        
                        # If better, update best configuration
                        if test_error < best_error:
                            best_error = test_error
                            best_config = test_config.copy()
                            improved = True
                            
                            if best_error <= orient_threshold:
                                print(f"Refinement achieved target orientation error: {best_error:.4f}")
                                break
        
        # If we improved enough, update the path
        if best_error <= orient_threshold:
            # Create a refined path by interpolating between the original path and our refined final configuration
            refined_path = path[:-1]  # Keep all but the last configuration
            
            # Add a few interpolated steps to ensure smooth transition
            steps = 5
            for step in range(1, steps + 1):
                t = step / steps
                interp_config = []
                for i in range(len(self.joint_indices)):
                    interp_val = path[-1][i] * (1-t) + best_config[i] * t
                    interp_config.append(interp_val)
                refined_path.append(interp_config)
            
            print(f"Local refinement successful. Final orientation error: {best_error:.4f}")
            return refined_path
        else:
            print(f"Local refinement unable to meet threshold. Best error: {best_error:.4f}")
            return None
    
    def cleanup(self):
        """Disconnect from PyBullet."""
        p.disconnect(self.client_id)

def main():
    # Define robot and joints of interest
    urdf_path = "devkit_base_descr.urdf"
    joint_names = ["pillar_platform_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    
    # Choose planner approach based on your needs
    # Option 1: With point cloud for collision minimization
    # point_cloud = o3d.io.read_point_cloud("your_point_cloud.pcd")
    # planner = MotionPlanner(urdf_path, joint_names, point_cloud=point_cloud, collision_free=False)
    
    # Option 2: Without point cloud, using PyBullet collision detection
    planner = MotionPlanner(urdf_path, joint_names, collision_free=True)
    
    # Print available links
    print("Available links in the robot:")
    for i in range(p.getNumJoints(planner.robot_id)):
        joint_info = p.getJointInfo(planner.robot_id, i)
        print(f"  Joint {i}: {joint_info[1].decode('utf-8')}, Link: {joint_info[12].decode('utf-8')}")
    
    # Set start configuration
    start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Set goal pose for end effector
    goal_link_name = "link6"
    goal_position = (-0.5, 0, 1.7)
    goal_orientation = (0.9, 0, -0.4, 0)
    
    # Find the goal link index by name
    goal_link_idx = None
    for i in range(p.getNumJoints(planner.robot_id)):
        joint_info = p.getJointInfo(planner.robot_id, i)
        link_name = joint_info[12].decode('utf-8')
        if link_name == goal_link_name:
            goal_link_idx = i
            print(f"Found goal link/joint at index {i}: {link_name}")
            break
    
    if goal_link_idx is None:
        raise ValueError(f"Goal link '{goal_link_name}' not found in URDF")
    
    # Set robot to start configuration
    for i, joint_idx in enumerate(planner.joint_indices):
        p.resetJointState(planner.robot_id, joint_idx, start_joints[i])
    
    # Get and print goal link position
    link_state = p.getLinkState(planner.robot_id, goal_link_idx)
    start_position = link_state[0]
    start_orientation = link_state[1]
    
    print(f"Goal link '{goal_link_name}' at index {goal_link_idx} in start configuration:")
    print(f"  Position: {start_position}")
    print(f"  Orientation: {start_orientation}")
    print(f"  Target Position: {goal_position}")
    print(f"  Target Orientation: {goal_orientation}")
    print(f"  Position Error: {[goal_position[i] - start_position[i] for i in range(3)]}")
    
    # Use the multi-attempt plan method with orientation refinement
    joint_trajectory = planner.plan(
        start_joints=start_joints,
        goal_link_name_or_idx=goal_link_idx,
        goal_position=goal_position,
        goal_orientation=goal_orientation,
        planning_time=5.0,  # Base planning time per attempt
        max_attempts=3      # Try up to 3 planning attempts
    )
    
    if joint_trajectory:
        print(f"Motion plan found with {len(joint_trajectory)} waypoints")
        # Print a sample of the path
        print("Sample of path waypoints:")
        for i in range(min(5, len(joint_trajectory))):
            print(f"  Waypoint {i}: {joint_trajectory[i]}")
        if len(joint_trajectory) > 10:
            print("  ...")
            for i in range(max(5, len(joint_trajectory)-5), len(joint_trajectory)):
                print(f"  Waypoint {i}: {joint_trajectory[i]}")
    
    # Clean up
    planner.cleanup()

def quaternion_distance(q1, q2):
    """
    Calculate the distance between two quaternions.
    Returns a value between 0 (identical) and 1 (opposite).
    """
    # Compute the dot product
    dot_product = abs(sum(q1[i] * q2[i] for i in range(4)))
    
    # Clamp the dot product to [-1, 1]
    dot_product = max(-1.0, min(dot_product, 1.0))
    
    # Distance is 1 - |dot|
    return 1.0 - dot_product

if __name__ == "__main__":
    main()
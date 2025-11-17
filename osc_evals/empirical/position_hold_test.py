"""
Position hold test - move to random positions and hold

Tests OSC controller by moving to random positions and holding them,
tracking steady-state error and other performance metrics.
"""

from typing import Optional, Iterator, Tuple, Dict
import numpy as np
import wandb

from .movement_test_base import MovementTestBase
from .metrics import TrackingMetrics
from .video_utils import VideoRecorder
from typing import List
from robosuite.utils import control_utils
import robosuite.utils.transform_utils as T

class PositionHoldTest(MovementTestBase):
    """Test: Move to random positions and hold for steady-state error analysis"""
    
    def __init__(self, env, render: bool = True, timestep: float = 0.01,
                 num_targets: int = 10, hold_duration: int = 125, use_wandb: bool = True,
                 workspace_bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                 target_positions: Optional[List[np.ndarray]] = None,
                 target_poses: Optional[List[Tuple[np.ndarray, np.ndarray]]] = None,
                 move_max_steps: int = 500,
                 log_videos: bool = True,
                 wandb_project: Optional[str] = None,
                 wandb_entity: Optional[str] = None,
                 wandb_run_name: Optional[str] = None,
                 wandb_config: Optional[Dict] = None):
        super().__init__(env, render, timestep)
        self.num_targets = num_targets
        self.hold_duration = hold_duration
        self.use_wandb = use_wandb
        self.workspace_bounds = workspace_bounds if workspace_bounds is not None else self._estimate_workspace_bounds()
        # Optional explicit target positions (list of np.ndarray shape (3,))
        self.target_positions = target_positions
        # Optional explicit target poses (list of tuples (pos, ori) where pos is (3,) and ori is (3,3) rotation matrix)
        self.target_poses = target_poses
        # Max steps allowed during movement phase before timing out
        self.move_max_steps = move_max_steps
        self.log_videos = log_videos
        self.wandb_project = wandb_project
        self.wandb_entity = wandb_entity
        self.wandb_run_name = wandb_run_name
        self.wandb_config = wandb_config or {}
        self._owns_wandb_run = False
        
    @property
    def name(self) -> str:
        return "Position Hold Test"
    
    @property
    def description(self) -> str:
        return "Move to random positions and hold to evaluate steady-state tracking error"
    
    def _estimate_workspace_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate workspace bounds based on current robot position"""
        if self.env is None:
            # Default bounds if no environment available
            return np.array([-0.3, -0.3, -0.3]), np.array([0.3, 0.3, 0.3])
        
        # Get initial end-effector position
        robot = self.env.robots[0]
        ee_positions = robot._hand_pos
        arm_name = robot.arms[0]
        initial_pos = ee_positions[arm_name]
        
        # Define workspace as a box around initial position
        # Different bounds for x, y, z to avoid going inside the table
        x_bounds = 0.4  # ±20cm in x
        y_bounds = 0.4  # ±20cm in y
        z_bounds_min = 0.05  # At least 5cm above initial position (avoid table)
        z_bounds_max = 0.25  # Up to 25cm above initial position
        
        min_bounds = np.array([
            initial_pos[0] - x_bounds,
            initial_pos[1] - y_bounds,
            initial_pos[2] + z_bounds_min
        ])
        max_bounds = np.array([
            initial_pos[0] + x_bounds,
            initial_pos[1] + y_bounds,
            initial_pos[2] + z_bounds_max
        ])
        
        return min_bounds, max_bounds
    
    def get_movement_pattern(self) -> Iterator[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """Yield either provided target poses/positions or generate random ones"""
        # If explicit pose targets provided (position + orientation), use them
        if isinstance(self.target_poses, list) and len(self.target_poses) > 0:
            for target_pos, target_ori in self.target_poses:
                yield target_pos, target_ori
            return
        
        # If explicit position targets provided, use them
        if isinstance(self.target_positions, list) and len(self.target_positions) > 0:
            for tp in self.target_positions:
                if self.is_pose_control:
                    yield tp, np.eye(3)
                else:
                    yield tp, None
            return
        
        # Else generate random
        np.random.seed(42)
        for _ in range(self.num_targets):
            min_bounds, max_bounds = self.workspace_bounds
            target_pos = np.random.uniform(min_bounds, max_bounds)
            if self.is_pose_control:
                yield target_pos, np.eye(3)
            else:
                yield target_pos, None
    
    def get_hold_duration(self) -> int:
        """Return the hold duration in timesteps"""
        return self.hold_duration
    
    def move_to_target(self, target_pos: np.ndarray, target_ori: Optional[np.ndarray] = None,
                     max_steps: int = 500, video_recorder=None, global_step=0) -> Tuple[bool, np.ndarray]:
        """Move to target position using proportional control"""
        reached = False
        action_history = []
        
        for step in range(max_steps):
            # Get current end-effector position
            current_pos, current_ori = self.get_ee_state()
            
            # Get controller to check reference frame
            robot = self.env.robots[0]
            arm_name = robot.arms[0]
            arm_controller = robot.part_controllers[arm_name]
            
            # Compute position error - need to handle coordinate frame correctly
            # If controller uses "base" frame, we need to transform error to base frame
            if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                # Transform both positions to base frame, then compute error
                # This ensures the error vector is in the correct frame
                target_pos_base = arm_controller.world_to_origin_frame(target_pos)
                current_pos_base = arm_controller.world_to_origin_frame(current_pos)
                pos_error = target_pos_base - current_pos_base
            else:
                # World frame - compute error directly
                pos_error = target_pos - current_pos
            
            # Debug output every 50 steps
            if step % 50 == 0:
                pos_error_mag = np.linalg.norm(pos_error)
                frame_info = ""
                controller_info = ""
                reachability_warning = ""
                ori_info = ""
                
                # Check if target is below table (assuming table is around z=0)
                if target_pos[2] < -0.05:  # Likely below table
                    reachability_warning = f" [WARNING: Target Z={target_pos[2]:.3f} may be below table!]"
                
                # Add orientation error information if pose control
                if target_ori is not None and self.is_pose_control:
                    # Compute orientation error in the same frame as the controller
                    if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                        # Transform orientations to base frame for error computation
                        origin_pose = T.make_pose(arm_controller.origin_pos, arm_controller.origin_ori)
                        origin_pose_inv = T.pose_inv(origin_pose)
                        target_pose_world = T.make_pose(np.zeros(3), target_ori)
                        target_pose_base = T.pose_in_A_to_pose_in_B(target_pose_world, origin_pose_inv)
                        target_ori_base = target_pose_base[:3, :3]
                        current_pose_world = T.make_pose(np.zeros(3), current_ori)
                        current_pose_base = T.pose_in_A_to_pose_in_B(current_pose_world, origin_pose_inv)
                        current_ori_base = current_pose_base[:3, :3]
                        ori_error = control_utils.orientation_error(target_ori_base, current_ori_base)
                    else:
                        ori_error = control_utils.orientation_error(target_ori, current_ori)
                    ori_error_mag = np.linalg.norm(ori_error)
                    # Convert to Euler angles for readability (in world frame for display)
                    target_euler = T.mat2euler(target_ori, axes="sxyz")
                    current_euler = T.mat2euler(current_ori, axes="sxyz")
                    ori_info = f", ori_error={ori_error_mag:.4f}rad ({np.degrees(ori_error_mag):.2f}°), "
                    ori_info += f"target_euler=[{target_euler[0]:.3f}, {target_euler[1]:.3f}, {target_euler[2]:.3f}], "
                    ori_info += f"current_euler=[{current_euler[0]:.3f}, {current_euler[1]:.3f}, {current_euler[2]:.3f}]"
                
                if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                    frame_info = f" (base_frame_error=[{pos_error[0]:.3f}, {pos_error[1]:.3f}, {pos_error[2]:.3f}])"
                    # Check controller's goal position
                    if hasattr(arm_controller, 'goal_pos') and arm_controller.goal_pos is not None:
                        goal_pos_base = arm_controller.goal_pos
                        # Transform goal back to world for display
                        if hasattr(arm_controller, 'origin_pos') and hasattr(arm_controller, 'origin_ori'):
                            # Transform goal_pos_base back to world frame
                            origin_pose = T.make_pose(arm_controller.origin_pos, arm_controller.origin_ori)
                            goal_pose_base = T.make_pose(goal_pos_base, np.eye(3))
                            goal_pose_world = T.pose_in_A_to_pose_in_B(goal_pose_base, origin_pose)
                            goal_pos_world, _ = T.mat2pose(goal_pose_world)
                            goal_error = np.linalg.norm(goal_pos_world - target_pos)
                            controller_info = f", controller_goal_world=[{goal_pos_world[0]:.3f}, {goal_pos_world[1]:.3f}, {goal_pos_world[2]:.3f}], goal_error={goal_error:.3f}m"
                print(f"    Step {step}: pos_error={pos_error_mag:.4f}m{frame_info}{controller_info}{reachability_warning}{ori_info}, "
                      f"current_pos=[{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}], "
                      f"target_pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            
            # Record video frame during movement (every 5 steps to avoid too many frames)
            if video_recorder is not None and step % 5 == 0:
                video_recorder.add_frame(
                    self.env,
                    pos_error[0], pos_error[1], pos_error[2],
                    global_step + step
                )
            
            # Create action - move toward target (saturate to max controller input)
            if self.is_pose_control:
                # For pose control, also handle orientation
                if target_ori is not None:
                    # Compute orientation error - need to handle coordinate frame correctly
                    # If controller uses "base" frame, we need to transform orientations to base frame
                    if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                        # Transform orientations to base frame
                        # Transform target_ori from world to base frame
                        origin_pose = T.make_pose(arm_controller.origin_pos, arm_controller.origin_ori)
                        origin_pose_inv = T.pose_inv(origin_pose)
                        target_pose_world = T.make_pose(np.zeros(3), target_ori)
                        target_pose_base = T.pose_in_A_to_pose_in_B(target_pose_world, origin_pose_inv)
                        target_ori_base = target_pose_base[:3, :3]
                        
                        # Transform current_ori from world to base frame
                        current_pose_world = T.make_pose(np.zeros(3), current_ori)
                        current_pose_base = T.pose_in_A_to_pose_in_B(current_pose_world, origin_pose_inv)
                        current_ori_base = current_pose_base[:3, :3]
                        
                        # Compute orientation error in base frame
                        ori_error = control_utils.orientation_error(target_ori_base, current_ori_base)
                    else:
                        # World frame - compute error directly
                        ori_error = control_utils.orientation_error(target_ori, current_ori)
                    
                    # Scale position and orientation errors separately before clipping
                    # This prevents orientation errors from affecting position control
                    action_xyz = pos_error * 20.0
                    
                    # Adaptive orientation scaling: scale based on error magnitude
                    # For small errors (< 0.1 rad), use higher scaling to get more response
                    # For larger errors, use moderate scaling to avoid overshooting
                    ori_error_mag = np.linalg.norm(ori_error)
                    if ori_error_mag < 0.05:  # Very small error (< 2.9°)
                        ori_scale = 20.0  # High scaling for fine control
                    elif ori_error_mag < 0.1:  # Small error (< 5.7°)
                        ori_scale = 15.0  # Moderate-high scaling
                    elif ori_error_mag < 0.2:  # Medium error (< 11.5°)
                        ori_scale = 10.0  # Moderate scaling
                    else:  # Large error
                        ori_scale = 5.0  # Lower scaling to avoid overshoot
                    
                    action_ori = ori_error * ori_scale
                    # Clip position and orientation separately to preserve position control
                    action_xyz = np.clip(action_xyz, -1.0, 1.0)
                    action_ori = np.clip(action_ori, -1.0, 1.0)
                    action_vec = np.concatenate([action_xyz, action_ori, [1]])
                else:
                    action_xyz = pos_error * 20.0
                    action_xyz = np.clip(action_xyz, -1.0, 1.0)
                    action_vec = np.concatenate([action_xyz, np.zeros(3), [1]])
            else:
                # Position only
                action_xyz = pos_error * 20.0
                action_xyz = np.clip(action_xyz, -1.0, 1.0)
                action_vec = np.concatenate([action_xyz, [1]])
            
            # Debug: show action being sent
            if step % 50 == 0 and step > 0:
                action_debug = f"      action_xyz (before clip)={pos_error * 20.0}, "
                action_debug += f"action_xyz (after clip)={action_xyz}, "
                if self.is_pose_control and target_ori is not None:
                    # Show orientation action if pose control
                    action_debug += f"action_ori (after clip)={action_ori}, "
                action_debug += f"action_vec={action_vec[:3] if len(action_vec) >= 3 else action_vec}"
                print(action_debug)
            
            # Step environment
            self.step(action_vec)
            action_history.append(action_vec)
            
            # Check if reached (within 3cm for position, and orientation error < 0.1 rad)
            pos_error_magnitude = np.linalg.norm(pos_error)
            reached_pos = pos_error_magnitude < 0.03
            
            if target_ori is not None and self.is_pose_control:
                # Compute orientation error in the same frame as used for actions
                if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                    # Use the same base-frame error computation as in action creation
                    origin_pose = T.make_pose(arm_controller.origin_pos, arm_controller.origin_ori)
                    origin_pose_inv = T.pose_inv(origin_pose)
                    target_pose_world = T.make_pose(np.zeros(3), target_ori)
                    target_pose_base = T.pose_in_A_to_pose_in_B(target_pose_world, origin_pose_inv)
                    target_ori_base = target_pose_base[:3, :3]
                    current_pose_world = T.make_pose(np.zeros(3), current_ori)
                    current_pose_base = T.pose_in_A_to_pose_in_B(current_pose_world, origin_pose_inv)
                    current_ori_base = current_pose_base[:3, :3]
                    ori_error = control_utils.orientation_error(target_ori_base, current_ori_base)
                else:
                    ori_error = control_utils.orientation_error(target_ori, current_ori)
                ori_error_magnitude = np.linalg.norm(ori_error)
                reached_ori = ori_error_magnitude < 0.05  # Stricter threshold: 0.05 rad (~2.9°)
                # Only require orientation if it's actually being controlled
                # For now, prioritize position - if position is reached, consider it reached
                # This prevents unreachable orientations from blocking position convergence
                reached = reached_pos  # Prioritize position convergence
                if reached_pos and not reached_ori:
                    # Log warning if position reached but orientation not
                    if step % 50 == 0:  # Only log occasionally to avoid spam
                        print(f"    Position reached but orientation error: {ori_error_magnitude:.3f} rad ({np.degrees(ori_error_magnitude):.2f}°)")
            else:
                reached = reached_pos
            
            if reached:
                break
        
        return reached, np.array(action_history)
    
    def run(self):
        """Execute the position hold test"""
        self.print_header()
        self.reset()
        
        # Get starting position
        robot = self.env.robots[0]
        ee_positions = robot._hand_pos
        arm_name = robot.arms[0]
        start_pos = ee_positions[arm_name]
        
        # Generate target positions using get_movement_pattern (handles both explicit and random)
        target_positions = list(self.get_movement_pattern())
        num_targets = len(target_positions)
        
        # Initialize wandb with correct target count
        if self.use_wandb:
            run_name = self.wandb_run_name or f"position_hold_{num_targets}_targets"
            project = self.wandb_project or "osc-controller-evals"
            base_config = {
                "num_targets": num_targets,
                "hold_duration": self.hold_duration,
                "controller_type": "OSC_POSE" if self.is_pose_control else "OSC_POSITION",
            }
            base_config.update(self.wandb_config)
            if wandb.run is None:
                wandb.init(
                    project=project,
                    entity=self.wandb_entity,
                    name=run_name,
                    config=base_config,
                )
                self._owns_wandb_run = True
            else:
                wandb.config.update(base_config, allow_val_change=True)
        
        print(f"Running position hold test with {num_targets} targets")
        print(f"Hold duration: {self.hold_duration} timesteps (~{self.hold_duration/20:.1f} seconds)")
        print(f"Starting position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
        print(f"Workspace bounds: {self.workspace_bounds[0]} to {self.workspace_bounds[1]}")
        print(f"Workspace range: X: ±{self.workspace_bounds[1][0] - start_pos[0]:.2f}m, Y: ±{self.workspace_bounds[1][1] - start_pos[1]:.2f}m, Z: +{self.workspace_bounds[1][2] - start_pos[2]:.2f}m\n")
        
        # Initialize video recorder (keep all history for complete error plot)
        video_recorder = VideoRecorder(max_history=None) if self.log_videos else None
        
        # Track all metrics
        all_segment_metrics = []
        overall_tracking = TrackingMetrics()
        
        global_step = 0
        
        # Run test for each target
        for target_idx, (target_pos, target_ori) in enumerate(target_positions):
            print(f"\nTarget {target_idx + 1}/{num_targets}: pos={target_pos}")
            
            # Print target orientation if available
            if target_ori is not None:
                target_euler = T.mat2euler(target_ori, axes="sxyz")
                print(f"  Target orientation (Euler): roll={target_euler[0]:.3f}, pitch={target_euler[1]:.3f}, yaw={target_euler[2]:.3f} rad")
                print(f"  Target orientation (degrees): roll={np.degrees(target_euler[0]):.2f}°, pitch={np.degrees(target_euler[1]):.2f}°, yaw={np.degrees(target_euler[2]):.2f}°")
            
            # Warn if target might be unreachable (below table)
            if target_pos[2] < -0.05:
                print(f"  WARNING: Target Z={target_pos[2]:.3f} is likely below the table surface (z≈0).")
                print(f"  The robot may not be able to reach this position due to collision constraints.")
            
            # Move to target (capture video during movement)
            print("  Moving to target...")
            reached, action_history = self.move_to_target(
                target_pos,
                target_ori,
                max_steps=self.move_max_steps,
                video_recorder=video_recorder,
                global_step=global_step,
            )
            print(f"  {'Reached' if reached else 'Timeout'} - holding position...")
            # Update global_step after movement - use actual number of steps taken
            actual_movement_steps = len(action_history)
            global_step += actual_movement_steps
            
            # Segment metrics
            segment_metrics = TrackingMetrics()
            
            # Hold at target
            # Get controller reference for hold phase
            robot = self.env.robots[0]
            arm_name = robot.arms[0]
            arm_controller = robot.part_controllers[arm_name]
            
            for hold_step in range(self.hold_duration):
                # Get current state
                current_pos, current_ori = self.get_ee_state()
                
                # Calculate errors
                pos_error = current_pos - target_pos
                
                # Record metrics - use target_ori (what we want) not desired_ori (what controller thinks)
                # This gives us the actual tracking error vs the target
                segment_metrics.add_observation(
                    target_pos, current_pos,
                    target_ori, current_ori,  # Use target orientation, not controller's desired
                    hold_step
                )
                overall_tracking.add_observation(
                    target_pos, current_pos,
                    target_ori, current_ori,  # Use target orientation, not controller's desired
                    global_step
                )
                
                # Record video frame
                if video_recorder is not None:
                    video_recorder.add_frame(
                        self.env,
                        pos_error[0], pos_error[1], pos_error[2],
                        global_step
                    )
                
                # Calculate orientation error if pose control
                ori_error_mag = None
                if target_ori is not None and self.is_pose_control:
                    # Compute orientation error in base frame if needed
                    if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                        origin_pose = T.make_pose(arm_controller.origin_pos, arm_controller.origin_ori)
                        origin_pose_inv = T.pose_inv(origin_pose)
                        target_pose_world = T.make_pose(np.zeros(3), target_ori)
                        target_pose_base = T.pose_in_A_to_pose_in_B(target_pose_world, origin_pose_inv)
                        target_ori_base = target_pose_base[:3, :3]
                        current_pose_world = T.make_pose(np.zeros(3), current_ori)
                        current_pose_base = T.pose_in_A_to_pose_in_B(current_pose_world, origin_pose_inv)
                        current_ori_base = current_pose_base[:3, :3]
                        ori_error = control_utils.orientation_error(target_ori_base, current_ori_base)
                    else:
                        ori_error = control_utils.orientation_error(target_ori, current_ori)
                    ori_error_mag = np.linalg.norm(ori_error)
                
                # Log to wandb
                if self.use_wandb:
                    log_dict = {
                        'step': global_step,
                        'segment': target_idx,
                        'position_error_l2': float(np.linalg.norm(pos_error)),
                        'position_error_x': float(pos_error[0]),
                        'position_error_y': float(pos_error[1]),
                        'position_error_z': float(pos_error[2]),
                    }
                    if ori_error_mag is not None:
                        log_dict['orientation_error'] = float(ori_error_mag)
                        log_dict['orientation_error_deg'] = float(np.degrees(ori_error_mag))
                    wandb.log(log_dict)
                
                # Continue sending small correction commands during hold phase
                # This helps maintain both position and orientation
                if self.is_pose_control and target_ori is not None:
                    # Compute position error in base frame if needed
                    if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                        target_pos_base = arm_controller.world_to_origin_frame(target_pos)
                        current_pos_base = arm_controller.world_to_origin_frame(current_pos)
                        pos_error_hold = target_pos_base - current_pos_base
                    else:
                        pos_error_hold = pos_error
                    
                    # Small position corrections
                    action_xyz = pos_error_hold * 10.0  # Reduced scaling for hold phase
                    action_xyz = np.clip(action_xyz, -0.5, 0.5)  # Smaller max action
                    
                    # Small orientation corrections
                    ori_error_mag_hold = np.linalg.norm(ori_error)
                    if ori_error_mag_hold < 0.05:
                        ori_scale_hold = 15.0  # Higher scaling for fine corrections
                    else:
                        ori_scale_hold = 10.0
                    action_ori = ori_error * ori_scale_hold
                    action_ori = np.clip(action_ori, -0.5, 0.5)  # Smaller max action
                    
                    action_vec = np.concatenate([action_xyz, action_ori, [1]])
                else:
                    # Position only - small corrections
                    if hasattr(arm_controller, 'input_ref_frame') and arm_controller.input_ref_frame == "base":
                        target_pos_base = arm_controller.world_to_origin_frame(target_pos)
                        current_pos_base = arm_controller.world_to_origin_frame(current_pos)
                        pos_error_hold = target_pos_base - current_pos_base
                    else:
                        pos_error_hold = pos_error
                    action_xyz = pos_error_hold * 10.0
                    action_xyz = np.clip(action_xyz, -0.5, 0.5)
                    action_vec = np.concatenate([action_xyz, [1]])
                
                self.step(action_vec)
                
                global_step += 1
            
            # Compute segment metrics
            segment_results = segment_metrics.get_all_metrics()
            all_segment_metrics.append(segment_results)
            
            # Log segment summary
            if self.use_wandb:
                log_dict = {
                    f'segment_{target_idx}/steady_state_error': segment_results['steady_state_error'],
                    f'segment_{target_idx}/settling_time': segment_results['settling_time'],
                    f'segment_{target_idx}/overshoot': segment_results['overshoot'],
                }
                # Add orientation metrics if available
                if 'orientation_error_mean' in segment_results:
                    log_dict[f'segment_{target_idx}/orientation_error_mean'] = segment_results['orientation_error_mean']
                    log_dict[f'segment_{target_idx}/orientation_error_max'] = segment_results['orientation_error_max']
                wandb.log(log_dict)
            
            # Print segment summary with orientation info
            summary_msg = f"  Segment complete. Steady-state pos_error: {segment_results['l2_error_mean']:.4f}m"
            if 'orientation_error_mean' in segment_results:
                summary_msg += f", ori_error: {segment_results['orientation_error_mean']:.4f}rad ({np.degrees(segment_results['orientation_error_mean']):.2f}°)"
            print(summary_msg)
        
        # Compute overall metrics
        overall_results = overall_tracking.get_all_metrics()
        
        # Create and log video
        if video_recorder and self.use_wandb:
            # Log just the robot frames directly (no error plot)
            if video_recorder.frames:
                robot_video = np.array(video_recorder.frames)
                if robot_video is not None and len(robot_video) > 0:
                    print(f"Robot video original shape: {robot_video.shape} (T, H, W, C)")
                    # Reshape from (T, H, W, C) to (T, C, H, W) for wandb
                    robot_video = np.transpose(robot_video, (0, 3, 1, 2))
                    print(f"Robot video reshaped to: {robot_video.shape} (T, C, H, W)")
                    wandb.log({"trajectory_video": wandb.Video(robot_video, fps=20, format="mp4")})
            
            # Also try with error plot
            video = video_recorder.create_combined_video()
            if video is not None:
                print(f"Combined video original shape: {video.shape} (T, H, W, C)")
                # Reshape from (T, H, W, C) to (T, C, H, W) for wandb
                video = np.transpose(video, (0, 3, 1, 2))
                print(f"Combined video reshaped to: {video.shape} (T, C, H, W)")
                wandb.log({"combined_video": wandb.Video(video, fps=20, format="mp4")})
        
        # Log final summary
        if self.use_wandb and wandb.run is not None:
            wandb.summary.update(overall_results)
            if self._owns_wandb_run:
                wandb.finish()
        
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print("="*60)
        print(f"Overall L2 Position Error: {overall_results['l2_error_mean']:.4f}m ± {overall_results['l2_error_std']:.4f}m")
        print(f"Max Position Error: {overall_results['l2_error_max']:.4f}m")
        print(f"Steady-state Position Error: {overall_results['steady_state_error']:.4f}m")
        
        # Print orientation metrics if available
        if 'orientation_error_mean' in overall_results:
            print(f"\nOrientation Tracking:")
            print(f"  Mean Orientation Error: {overall_results['orientation_error_mean']:.4f}rad ({np.degrees(overall_results['orientation_error_mean']):.2f}°)")
            print(f"  Max Orientation Error: {overall_results['orientation_error_max']:.4f}rad ({np.degrees(overall_results['orientation_error_max']):.2f}°)")
            print(f"  Std Orientation Error: {overall_results['orientation_error_std']:.4f}rad ({np.degrees(overall_results['orientation_error_std']):.2f}°)")
        else:
            print("\nNote: Orientation tracking not available (using OSC_POSITION or no orientation targets)")

        return overall_results


"""
Evaluate by converting joint targets to end-effector positions and using PositionHoldTest.

Expected target vector order (length 13):
    [torso_waist_yaw, torso_waist_pitch, torso_waist_roll,
     head_yaw, head_roll, head_pitch,
     r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw,
     r_elbow_pitch, r_wrist_yaw, r_wrist_roll, r_wrist_pitch]
"""

from typing import List, Optional, Iterator, Tuple
import numpy as np
import wandb

from .movement_test_base import MovementTestBase
from .metrics import TrackingMetrics
from .video_utils import VideoRecorder


class JointTargetsEval(MovementTestBase):
    """Convert joint targets to end-effector positions and use PositionHoldTest framework"""

    def __init__(self, env, render: bool = True, timestep: float = 0.01,
                 targets: Optional[List[np.ndarray]] = None, hold_duration: int = 125, 
                 use_wandb: bool = True, move_max_steps: int = 500):
        super().__init__(env, render, timestep)
        self.targets = targets or []
        self.hold_duration = hold_duration
        self.use_wandb = use_wandb
        self.move_max_steps = move_max_steps

        # Names in the expected incoming order
        self.joint_names_order = [
            "torso_waist_yaw", "torso_waist_pitch", "torso_waist_roll",
            "head_yaw", "head_roll", "head_pitch",
            "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
            "r_elbow_pitch", "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
        ]
        self.qpos_indices = None
        self.ee_targets = None  # Will store converted end-effector positions

    @property
    def name(self) -> str:
        return "Joint Targets Eval"

    @property
    def description(self) -> str:
        return "Convert joint targets to end-effector positions and evaluate with PositionHoldTest metrics"

    def _compute_qpos_indices(self):
        """Map the ordered joint names to sim qpos indices (once)."""
        if self.qpos_indices is not None:
            return
        sim = self.env.sim
        indices = []
        for jn in self.joint_names_order:
            try:
                idx = sim.model.get_joint_qpos_addr(jn)
                indices.append(idx)
            except Exception:
                indices.append(None)
        self.qpos_indices = indices

    def _joint_target_to_ee_pos(self, target: np.ndarray):
        """Convert joint target to end-effector position by setting joints and reading EE pos"""
        sim = self.env.sim
        self._compute_qpos_indices()
        
        # Save current state
        old_qpos = sim.data.qpos.copy()
        
        # Apply joint target
        for jval, jidx in zip(target, self.qpos_indices):
            if jidx is not None:
                sim.data.qpos[jidx] = jval
        sim.forward()
        
        # Read end-effector position
        robot = self.env.robots[0]
        arm_name = robot.arms[0]
        ee_pos = robot._hand_pos[arm_name]
        
        # Restore old state
        sim.data.qpos[:] = old_qpos
        sim.forward()
        
        return ee_pos

    def get_movement_pattern(self) -> Iterator[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """Convert joint targets to end-effector positions"""
        if not self.targets:
            return
        
        # Convert all joint targets to EE positions
        if self.ee_targets is None:
            print("Converting joint targets to end-effector positions...")
            self.ee_targets = []
            for i, joint_target in enumerate(self.targets):
                ee_pos = self._joint_target_to_ee_pos(joint_target)
                self.ee_targets.append(ee_pos)
                print(f"  Target {i+1}: EE pos = [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        
        # Yield EE positions (and orientation if pose control)
        for ee_pos in self.ee_targets:
            if self.is_pose_control:
                # Use identity rotation for now (can be improved later)
                target_ori = np.eye(3)
            else:
                target_ori = None
            yield ee_pos, target_ori

    def get_hold_duration(self) -> int:
        """Return the hold duration in timesteps"""
        return self.hold_duration

    def run(self):
        """Execute the joint targets test using PositionHoldTest framework"""
        # Initialize wandb
        if self.use_wandb:
            wandb.init(
                project="osc-controller-evals",
                name=f"joint_targets_{len(self.targets)}_targets",
                config={
                    "num_targets": len(self.targets),
                    "hold_duration": self.hold_duration,
                    "controller_type": "OSC_POSE" if self.is_pose_control else "OSC_POSITION",
                }
            )
        
        self.print_header()
        self.reset()
        
        if not self.targets:
            print("No joint targets provided; nothing to do.")
            return
        
        # Get starting position
        robot = self.env.robots[0]
        ee_positions = robot._hand_pos
        arm_name = robot.arms[0]
        start_pos = ee_positions[arm_name]
        
        print(f"Running joint targets eval with {len(self.targets)} targets")
        print(f"Hold duration: {self.hold_duration} timesteps (~{self.hold_duration/20:.1f} seconds)")
        print(f"Starting position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]\n")
        
        # Generate target positions (converts joints to EE pos)
        target_positions = list(self.get_movement_pattern())
        
        # Initialize video recorder
        video_recorder = VideoRecorder(max_history=None)
        
        # Track all metrics
        all_segment_metrics = []
        overall_tracking = TrackingMetrics()
        
        global_step = 0
        
        # Run test for each target
        for target_idx, (target_pos, target_ori) in enumerate(target_positions):
            print(f"\nTarget {target_idx + 1}/{len(target_positions)}: {target_pos}")
            
            # Move to target (from PositionHoldTest)
            print("  Moving to target...")
            reached, _ = self.move_to_target(
                target_pos,
                target_ori,
                max_steps=self.move_max_steps,
                video_recorder=video_recorder,
                global_step=global_step,
            )
            print(f"  {'Reached' if reached else 'Timeout'} - holding position...")
            global_step += 200  # Approximate movement steps
            
            # Segment metrics
            segment_metrics = TrackingMetrics()
            
            # Hold at target
            for hold_step in range(self.hold_duration):
                # Get current state
                current_pos, current_ori = self.get_ee_state()
                desired_pos_controller = self.get_desired_ee_position()
                desired_pos = desired_pos_controller if desired_pos_controller is not None else target_pos
                desired_ori = self.get_desired_ee_orientation() if self.is_pose_control else None
                
                # Calculate errors
                pos_error = current_pos - target_pos
                
                # Record metrics
                segment_metrics.add_observation(
                    target_pos, current_pos,
                    desired_ori, current_ori,
                    hold_step
                )
                overall_tracking.add_observation(
                    target_pos, current_pos,
                    desired_ori, current_ori,
                    global_step
                )
                
                # Record video frame
                video_recorder.add_frame(
                    self.env,
                    pos_error[0], pos_error[1], pos_error[2],
                    global_step
                )
                
                # Log to wandb
                if self.use_wandb:
                    wandb.log({
                        'step': global_step,
                        'segment': target_idx,
                        'position_error_l2': float(np.linalg.norm(pos_error)),
                        'position_error_x': float(pos_error[0]),
                        'position_error_y': float(pos_error[1]),
                        'position_error_z': float(pos_error[2]),
                    })
                
                # Maintain position
                action_vec = np.array([0, 0, 0, 1]) if not self.is_pose_control else np.array([0, 0, 0, 0, 0, 0, 1])
                self.step(action_vec)
                
                global_step += 1
            
            # Compute segment metrics
            segment_results = segment_metrics.get_all_metrics()
            all_segment_metrics.append(segment_results)
            
            # Log segment summary
            if self.use_wandb:
                wandb.log({
                    f'segment_{target_idx}/steady_state_error': segment_results['steady_state_error'],
                    f'segment_{target_idx}/settling_time': segment_results['settling_time'],
                    f'segment_{target_idx}/overshoot': segment_results['overshoot'],
                })
            
            print(f"  Segment complete. Steady-state error: {segment_results['l2_error_mean']:.4f}m")
        
        # Compute overall metrics
        overall_results = overall_tracking.get_all_metrics()
        
        # Create and log video
        if video_recorder and self.use_wandb:
            if video_recorder.frames:
                robot_video = np.array(video_recorder.frames)
                if robot_video is not None and len(robot_video) > 0:
                    robot_video = np.transpose(robot_video, (0, 3, 1, 2))
                    wandb.log({"trajectory_video": wandb.Video(robot_video, fps=20, format="mp4")})
            
            video = video_recorder.create_combined_video()
            if video is not None:
                video = np.transpose(video, (0, 3, 1, 2))
                wandb.log({"combined_video": wandb.Video(video, fps=20, format="mp4")})
        
        # Log final summary
        if self.use_wandb:
            wandb.summary.update(overall_results)
            wandb.finish()
        
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print("="*60)
        print(f"Overall L2 Error: {overall_results['l2_error_mean']:.4f}m ± {overall_results['l2_error_std']:.4f}m")
        print(f"Max Error: {overall_results['l2_error_max']:.4f}m")
        print(f"Steady-state Error: {overall_results['steady_state_error']:.4f}m")



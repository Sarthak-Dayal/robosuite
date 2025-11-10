"""
Main CLI interface for OSC evaluations
"""

import argparse
from .environment import OSCEnvironmentManager
from .runner import DemoRunner
from .registry import DEMO_REGISTRY

# Ensure all demos are registered before parsing arguments
from . import visual
from . import empirical


def main():
    """Main entry point for OSC evaluations"""
    parser = argparse.ArgumentParser(
        description="OSC Controller Evaluations - Visual & Empirical",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Available Demos:
{chr(10).join(f"  {name:12s} - {demo_class(None).description.split(chr(10))[0]}" 
              for name, demo_class in DEMO_REGISTRY.items())}

Examples:
  python demo_osc_controller.py --demo square
  python demo_osc_controller.py --demo circle --robot UR5e
  python demo_osc_controller.py --demo impedance --controller OSC_POSE
        """
    )
    
    parser.add_argument(
        "--demo",
        type=str,
        default="square",
        choices=list(DEMO_REGISTRY.keys()),
        help="Which demo to run"
    )
    parser.add_argument(
        "--controller",
        type=str,
        default="OSC_POSE",
        choices=["OSC_POSE", "OSC_POSITION"],
        help="Controller type to use (may be overridden by demo requirements)"
    )
    parser.add_argument(
        "--impedance",
        type=str,
        default="fixed",
        choices=["fixed", "variable", "variable_kp"],
        help="Impedance mode (may be overridden by demo requirements)"
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="Panda",
        help="Robot to use"
    )
    parser.add_argument(
        "--env",
        type=str,
        default="Lift",
        help="Environment to use"
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Disable rendering"
    )
    parser.add_argument(
        "--wandb",
        action="store_true",
        help="Enable wandb logging (required for empirical tests)"
    )
    parser.add_argument(
        "--num-targets",
        type=int,
        default=10,
        help="Number of target positions for position hold test (use 1-3 for quick testing)"
    )
    parser.add_argument(
        "--hold-duration",
        type=int,
        default=125,
        help="Hold duration in timesteps for position hold test (use 25-50 for quick testing)"
    )
    parser.add_argument(
        "--move-max-steps",
        type=int,
        default=500,
        help="Maximum timesteps allowed to reach each target before timing out"
    )
    parser.add_argument(
        "--workspace-x-range",
        type=float,
        default=None,
        help="X-axis workspace range (meters from center, e.g., 0.4 means ±0.4m)"
    )
    parser.add_argument(
        "--workspace-y-range",
        type=float,
        default=None,
        help="Y-axis workspace range (meters from center, e.g., 0.4 means ±0.4m)"
    )
    parser.add_argument(
        "--workspace-z-min",
        type=float,
        default=None,
        help="Z-axis minimum offset from initial position (meters, e.g., 0.05)"
    )
    parser.add_argument(
        "--workspace-z-max",
        type=float,
        default=None,
        help="Z-axis maximum offset from initial position (meters, e.g., 0.25)"
    )
    parser.add_argument(
        "--position-targets-json",
        type=str,
        default=None,
        help="Path to JSON file with target EE positions for position_hold (list of [x,y,z])"
    )
    parser.add_argument(
        "--position-joint-targets-json",
        type=str,
        default=None,
        help="Path to JSON file with joint targets (length-13 arrays) to be converted to EE positions for position_hold"
    )
    parser.add_argument(
        "--pose-targets-json",
        type=str,
        default=None,
        help="Path to JSON file with target EE poses for position_hold (list of [x,y,z,roll,pitch,yaw] arrays)"
    )
    # Joint targets eval specific
    parser.add_argument(
        "--targets-json",
        type=str,
        default=None,
        help="Path to JSON file with joint targets (list of lists, each length 13) for joint_targets demo"
    )
    
    args = parser.parse_args()
    
    # Print startup info
    print("\n" + "="*60)
    print("ROBOSUITE OSC CONTROLLER DEMO - MODULAR FRAMEWORK")
    print("="*60)
    print(f"Environment: {args.env}")
    print(f"Robot: {args.robot}")
    print(f"Controller: {args.controller}")
    print(f"Impedance Mode: {args.impedance}")
    print(f"Demo: {args.demo}")
    
    # Create environment manager
    env_manager = OSCEnvironmentManager(
        robot=args.robot,
        environment=args.env
    )
    
    # Create initial environment for info display
    env = env_manager.create_environment(
        controller_type=args.controller,
        impedance_mode=args.impedance,
        use_offscreen=args.no_render,
        use_camera_obs=False,
    )
    
    env_manager.print_controller_info(env)
    
    # Run demo
    demo_runner = DemoRunner(env_manager)
    
    try:
        demo_class = DEMO_REGISTRY[args.demo]
        
        # Prepare kwargs for empirical tests
        demo_kwargs = {}
        if args.wandb:
            demo_kwargs['use_wandb'] = True
            demo_kwargs['num_targets'] = args.num_targets
            demo_kwargs['hold_duration'] = args.hold_duration
            # Add workspace bounds if specified
            if any([args.workspace_x_range, args.workspace_y_range, args.workspace_z_min, args.workspace_z_max]):
                import numpy as np
                # Create a temporary env to get initial position
                temp_env = env_manager.create_environment(
                    controller_type=args.controller,
                    impedance_mode=args.impedance,
                    use_offscreen=True,
                    use_camera_obs=False,
                )
                robot = temp_env.robots[0]
                ee_positions = robot._hand_pos
                arm_name = robot.arms[0]
                initial_pos = ee_positions[arm_name]
                env_manager.close()
                
                # Use specified values or defaults
                x_range = args.workspace_x_range if args.workspace_x_range is not None else 0.4
                y_range = args.workspace_y_range if args.workspace_y_range is not None else 0.4
                z_min = args.workspace_z_min if args.workspace_z_min is not None else 0.05
                z_max = args.workspace_z_max if args.workspace_z_max is not None else 0.25
                
                min_bounds = np.array([
                    initial_pos[0] - x_range,
                    initial_pos[1] - y_range,
                    initial_pos[2] + z_min
                ])
                max_bounds = np.array([
                    initial_pos[0] + x_range,
                    initial_pos[1] + y_range,
                    initial_pos[2] + z_max
                ])
                demo_kwargs['workspace_bounds'] = (min_bounds, max_bounds)
        
        # Special handling for position_hold: explicit EE poses, positions, or convert from joint targets
        if args.demo == 'position_hold':
            import json, numpy as np, os
            from robosuite.utils import transform_utils as T
            
            # Case 1: explicit EE poses (position + orientation) [x, y, z, roll, pitch, yaw]
            if args.pose_targets_json and os.path.exists(args.pose_targets_json):
                with open(args.pose_targets_json, 'r') as f:
                    pose_data = json.load(f)
                
                # Convert each pose [x, y, z, roll, pitch, yaw] to (position, orientation_matrix)
                # NOTE: euler2mat expects [roll, pitch, yaw] but uses ZYX convention internally
                # The function negates and reverses the order, so [r, p, y] becomes [-y, -p, -r] internally
                target_poses = []
                for i, pose in enumerate(pose_data):
                    if len(pose) != 6:
                        raise ValueError(f"Pose target must be [x, y, z, roll, pitch, yaw] (6 elements), got {len(pose)} elements")
                    pos = np.array(pose[:3], dtype=float)
                    euler = np.array(pose[3:6], dtype=float)  # roll, pitch, yaw (in radians)
                    ori_mat = T.euler2mat(euler)  # Convert to rotation matrix
                    target_poses.append((pos, ori_mat))
                    print(f"[CLI] Target {i+1}: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                          f"euler=[{euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f}] rad "
                          f"(roll, pitch, yaw)")
                
                demo_kwargs['target_poses'] = target_poses
                print(f"[CLI] Loaded {len(target_poses)} pose targets from {args.pose_targets_json}")
            
            # Case 2: explicit EE positions
            elif args.position_targets_json and os.path.exists(args.position_targets_json):
                with open(args.position_targets_json, 'r') as f:
                    pos_data = json.load(f)
                demo_kwargs['target_positions'] = [np.array(p, dtype=float) for p in pos_data]
            # Case 3: provide joint targets to convert into EE positions
            elif args.position_joint_targets_json and os.path.exists(args.position_joint_targets_json):
                with open(args.position_joint_targets_json, 'r') as f:
                    joint_data = json.load(f)
                joint_targets = [np.array(j, dtype=float) for j in joint_data]

                # Build a temporary offscreen env to compute FK
                temp_env = env_manager.create_environment(
                    controller_type=args.controller,
                    impedance_mode=args.impedance,
                    use_offscreen=True,
                    use_camera_obs=False,
                )
                sim = temp_env.sim
                robot = temp_env.robots[0]
                arm_name = robot.arms[0]

                joint_names_order = [
                    "torso_waist_yaw", "torso_waist_pitch", "torso_waist_roll",
                    "head_yaw", "head_roll", "head_pitch",
                    "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
                    "r_elbow_pitch", "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
                ]
                # Map names to qpos indices (missing joints allowed)
                qpos_indices = []
                for jn in joint_names_order:
                    try:
                        qpos_indices.append(sim.model.get_joint_qpos_addr(jn))
                    except Exception:
                        qpos_indices.append(None)

                # Convert all joint targets to EE positions
                old_qpos = sim.data.qpos.copy()
                ee_positions = []
                for jt in joint_targets:
                    # Apply joint target
                    for val, idx in zip(jt, qpos_indices):
                        if idx is not None:
                            sim.data.qpos[idx] = float(val)
                    sim.forward()
                    ee_pos = robot._hand_pos[arm_name]
                    ee_positions.append([float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])])
                    # Restore state
                    sim.data.qpos[:] = old_qpos
                    sim.forward()

                # Close temp env
                env_manager.close()

                # Persist converted positions to JSON for reproducibility
                out_path = os.path.join(os.path.dirname(__file__), "converted_ee_targets.json")
                with open(out_path, 'w') as f:
                    json.dump(ee_positions, f, indent=2)
                print(f"[CLI] Wrote converted EE targets to: {out_path}")

                # Provide to demo
                demo_kwargs['target_positions'] = [np.array(p, dtype=float) for p in ee_positions]

            # Movement time limit for reaching targets
            demo_kwargs['move_max_steps'] = args.move_max_steps

        # Special handling for joint_targets demo
        if args.demo == 'joint_targets':
            import numpy as np, json, os
            if args.targets_json and os.path.exists(args.targets_json):
                with open(args.targets_json, 'r') as f:
                    data = json.load(f)
                targets = [np.array(arr, dtype=float) for arr in data]
            else:
                # Default hardcoded targets (torso+head+right arm), each len=13
                targets = [
                    np.array([ 0.008, -0.003,  0.051, -0.000, -0.000, -0.000,  0.041, -0.738, -0.726, -0.123,  0.970,  0.412, -1.096]),
                    np.array([ 0.012, -0.020, -0.001, -0.000, -0.000,  0.000, -1.707,  0.000, -0.124,  0.000, -0.012,  1.072,  1.328]),
                    np.array([ 0.003,  0.024, -0.022, -0.000,  0.000, -0.000, -1.621, -0.661, -0.975, -0.152,  2.073,  0.228,  1.153]),
                    np.array([-0.066, -0.130,  0.136,  0.000, -0.001, -0.001,  0.973, -0.841, -0.053, -0.016,  0.450,  0.268, -1.018]),
                ]
            demo_kwargs['targets'] = targets
            # Provide a slightly longer hold for clear visualization
            demo_kwargs['hold_duration'] = args.hold_duration if args.hold_duration else 150
            demo_kwargs['move_max_steps'] = args.move_max_steps
            # Enable wandb if flag is set
            demo_kwargs['use_wandb'] = args.wandb

        # Run with explicit error logging
        import traceback
        try:
            print("\n[CLI] Starting demo run...\n")
            print(f"[CLI] Demo       : {args.demo}")
            print(f"[CLI] Robot      : {args.robot}")
            print(f"[CLI] Controller : {args.controller}")
            if args.demo == 'joint_targets':
                print(f"[CLI] Targets    : {len(demo_kwargs.get('targets', []))} provided")
            demo_runner.run_demo(
                demo_class=demo_class,
                controller_type=args.controller,
                impedance_mode=args.impedance,
                render=not args.no_render,
                **demo_kwargs
            )
            print("\n[CLI] Demo run completed successfully.\n")
        except Exception as e:
            print("\n[CLI ERROR] Demo run failed with exception:\n")
            traceback.print_exc()
            raise
        
        print("\n" + "="*60)
        print("DEMO COMPLETE!")
        print("="*60)
        print(f"\nAvailable demos: {', '.join(DEMO_REGISTRY.keys())}")
        print("For more info, see: osc_controller_explanation.md\n")
        
    finally:
        demo_runner.cleanup()

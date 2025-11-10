"""
Environment management for OSC demos
"""

from typing import Dict, Any
import os
import sys
import threading
import robosuite as suite
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config

# Set MUJOCO_GL early for macOS headless rendering if needed
if sys.platform == "darwin":
    # Check if we're on the main thread - if not, force offscreen rendering
    try:
        is_main_thread = threading.current_thread() is threading.main_thread()
    except AttributeError:
        # Fallback for older Python versions
        is_main_thread = threading.current_thread().name == "MainThread"
    
    if not is_main_thread:
        # Force osmesa backend to avoid GLFW window creation on non-main thread
        if os.environ.get("MUJOCO_GL") is None:
            os.environ["MUJOCO_GL"] = "osmesa"
            print("[Environment] Running on non-main thread on macOS; set MUJOCO_GL=osmesa for headless rendering.")


class OSCEnvironmentManager:
    """Manages OSC environment creation and configuration"""
    
    def __init__(self, robot: str = "Panda", environment: str = "Lift",
                 control_freq: int = 20, horizon: int = 1000):
        self.robot = robot
        self.environment = environment
        self.control_freq = control_freq
        self.horizon = horizon
        self._env = None
    
    def create_environment(self, controller_type: str = "OSC_POSE",
                          impedance_mode: str = "fixed",
                          use_offscreen: bool = False,
                          use_camera_obs: bool = False) -> Any:
        """Create and return a configured environment"""
        # Load the part controller config
        arm_controller_config = suite.load_part_controller_config(
            default_controller=controller_type
        )
        
        # Modify impedance mode if needed
        if impedance_mode != "fixed":
            arm_controller_config["impedance_mode"] = impedance_mode
        
        # Wrap it in composite controller format for the robot
        controller_config = refactor_composite_controller_config(
            arm_controller_config,
            robot_type=self.robot,
            arms=["right"]
        )
        
        # Create environment
        # Use "mujoco" renderer for headless/offscreen; "mjviewer" for on-screen
        # On macOS, GLFW windows must be created on the main thread. If not, force offscreen.
        if sys.platform == "darwin":
            try:
                is_main_thread = threading.current_thread() is threading.main_thread()
            except AttributeError:
                # Fallback for older Python versions
                is_main_thread = threading.current_thread().name == "MainThread"
            
            if not is_main_thread:
                if not use_offscreen:
                    print("[Environment] Not on main thread on macOS; forcing offscreen rendering to avoid NSWindow crash.")
                    use_offscreen = True
                # Ensure MUJOCO_GL is set (should already be set at module level, but double-check)
                if os.environ.get("MUJOCO_GL") is None:
                    os.environ["MUJOCO_GL"] = "osmesa"
                    print("[Environment] Set MUJOCO_GL=osmesa for headless rendering on macOS.")
        
        # Always use "mujoco" renderer when offscreen to avoid any window creation
        renderer_backend = "mujoco" if use_offscreen else "mjviewer"
        # Ensure has_renderer is False when offscreen to prevent any window creation
        has_renderer = False if use_offscreen else True
        
        self._env = suite.make(
            self.environment,
            robots=self.robot,
            controller_configs=controller_config,
            has_renderer=has_renderer,
            has_offscreen_renderer=use_offscreen,
            use_camera_obs=use_camera_obs,
            camera_names="frontview" if use_camera_obs else None,
            camera_heights=512 if use_camera_obs else 256,
            camera_widths=512 if use_camera_obs else 256,
            control_freq=self.control_freq,
            horizon=self.horizon,
            ignore_done=True,  # Prevent episodes from terminating early
            renderer=renderer_backend,
        )
        
        return self._env
    
    def get_controller_info(self, env) -> Dict[str, Any]:
        """Extract controller information from environment"""
        robot = env.robots[0]
        arm_name = robot.arms[0]
        arm_controller = robot.part_controllers[arm_name]
        
        low, high = arm_controller.control_limits
        
        return {
            'name': arm_controller.name,
            'control_dim': arm_controller.control_dim,
            'input_type': arm_controller.input_type,
            'input_ref_frame': arm_controller.input_ref_frame,
            'impedance_mode': arm_controller.impedance_mode,
            'kp': arm_controller.kp,
            'kd': arm_controller.kd,
            'use_ori': arm_controller.use_ori,
            'uncoupling': arm_controller.uncoupling,
            'control_limits': (low, high),
            'output_max': arm_controller.output_max,
            'output_min': arm_controller.output_min,
        }
    
    def print_controller_info(self, env):
        """Print detailed controller information"""
        info = self.get_controller_info(env)
        
        print("\n" + "="*60)
        print("CONTROLLER INFORMATION")
        print("="*60)
        print(f"Controller Type: {info['name']}")
        
        print(f"\nArm Controller Details:")
        print(f"  Type: {info['name']}")
        print(f"  Control Dimension: {info['control_dim']}")
        print(f"  Input Type: {info['input_type']}")
        print(f"  Input Reference Frame: {info['input_ref_frame']}")
        print(f"  Impedance Mode: {info['impedance_mode']}")
        print(f"  KP (stiffness): {info['kp']}")
        print(f"  KD (damping): {info['kd']}")
        print(f"  Control Orientation: {info['use_ori']}")
        print(f"  Uncouple Pos/Ori: {info['uncoupling']}")
        
        low, high = info['control_limits']
        print(f"\nAction Space:")
        print(f"  Shape: ({len(low)},)")
        print(f"  Low: {low}")
        print(f"  High: {high}")
        print(f"  Output Max (m/rad per step): {info['output_max']}")
        print(f"  Output Min (m/rad per step): {info['output_min']}")
        
        print("="*60 + "\n")
    
    def close(self):
        """Close the current environment"""
        if self._env is not None:
            self._env.close()
            self._env = None

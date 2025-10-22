"""
Environment management for OSC demos
"""

from typing import Dict, Any
import robosuite as suite
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config


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
                          impedance_mode: str = "fixed") -> Any:
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
        self._env = suite.make(
            self.environment,
            robots=self.robot,
            controller_configs=controller_config,
            has_renderer=True,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=self.control_freq,
            horizon=self.horizon,
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

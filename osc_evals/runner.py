"""
Demo execution runner
"""

from typing import Any, Dict

from .base import OSCDemoBase
from .environment import OSCEnvironmentManager


class DemoRunner:
    """Manages demo execution with automatic environment configuration"""
    
    def __init__(self, env_manager: OSCEnvironmentManager):
        self.env_manager = env_manager
        self.current_env = None
    
    def run_demo(self, demo_class: type[OSCDemoBase], 
                 controller_type: str, impedance_mode: str,
                 render: bool = True,
                 controller_overrides: Dict[str, Any] = None,
                 **kwargs):
        """Run a demo with appropriate environment configuration"""
        
        # Check if we need to override controller settings
        # Create a temporary instance to check requirements
        temp_demo = demo_class(None)
        required_controller = temp_demo.required_controller_type
        required_impedance = temp_demo.required_impedance_mode
        
        if required_controller and required_controller != controller_type:
            print(f"Note: This demo requires {required_controller}")
            print(f"Overriding controller type to {required_controller}...")
            controller_type = required_controller
        
        if required_impedance and required_impedance != impedance_mode:
            print(f"Note: This demo requires {required_impedance} impedance mode")
            print(f"Overriding impedance mode to {required_impedance}...")
            impedance_mode = required_impedance
        
        # Create environment with appropriate settings
        if self.current_env is not None:
            self.env_manager.close()
        
        # Determine offscreen rendering:
        # - Force offscreen if window rendering is disabled (render == False)
        # - Or if wandb logging is enabled (for video frames)
        use_offscreen = (not render) or kwargs.get('use_wandb', False)
        # Use camera obs only if offscreen rendering is enabled
        use_camera_obs = use_offscreen
        
        # Increase horizon for empirical tests to avoid early termination
        if use_offscreen:
            # For empirical tests, we need longer horizons
            # Each target: ~200 steps to reach + 125 steps to hold = 325 steps
            # 10 targets = 3250 steps minimum
            self.env_manager.horizon = 5000
        
        self.current_env = self.env_manager.create_environment(
            controller_type=controller_type,
            impedance_mode=impedance_mode,
            use_offscreen=use_offscreen,
            use_camera_obs=use_camera_obs,
            controller_overrides=controller_overrides,
        )
        
        # Disable window rendering whenever offscreen is enabled
        actual_render = render and not use_offscreen
        
        # Create and run demo, passing through kwargs for empirical test parameters
        demo = demo_class(self.current_env, render=actual_render, **kwargs)
        return demo.run()
    
    def cleanup(self):
        """Clean up resources"""
        self.env_manager.close()

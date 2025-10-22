"""
Demo execution runner
"""

from .base import OSCDemoBase
from .environment import OSCEnvironmentManager


class DemoRunner:
    """Manages demo execution with automatic environment configuration"""
    
    def __init__(self, env_manager: OSCEnvironmentManager):
        self.env_manager = env_manager
        self.current_env = None
    
    def run_demo(self, demo_class: type[OSCDemoBase], 
                 controller_type: str, impedance_mode: str,
                 render: bool = True):
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
        
        self.current_env = self.env_manager.create_environment(
            controller_type=controller_type,
            impedance_mode=impedance_mode
        )
        
        # Create and run demo
        demo = demo_class(self.current_env, render=render)
        demo.run()
    
    def cleanup(self):
        """Clean up resources"""
        self.env_manager.close()

"""
Base class for OSC demonstrations
"""

from abc import ABC, abstractmethod
from typing import Optional
import time


class OSCDemoBase(ABC):
    """Base class for OSC demonstrations"""
    
    def __init__(self, env, render: bool = True, timestep: float = 0.01):
        self.env = env
        self.render_enabled = render
        self.timestep = timestep
        
        # Extract controller info (only if env is not None)
        if env is not None:
            robot = env.robots[0]
            self.arm_name = robot.arms[0]
            self.arm_controller = robot.part_controllers[self.arm_name]
            self.is_pose_control = self.arm_controller.name == "OSC_POSE"
        else:
            # Placeholder values when env is None (e.g., for help text generation)
            self.arm_name = None
            self.arm_controller = None
            self.is_pose_control = False
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Return the demo name"""
        pass
    
    @property
    @abstractmethod
    def description(self) -> str:
        """Return a brief description of the demo"""
        pass
    
    @property
    def required_controller_type(self) -> Optional[str]:
        """Return required controller type (OSC_POSE/OSC_POSITION) or None if flexible"""
        return None
    
    @property
    def required_impedance_mode(self) -> Optional[str]:
        """Return required impedance mode or None if flexible"""
        return None
    
    def print_header(self):
        """Print demo header"""
        print("\n" + "="*60)
        print(f"{self.name.upper()}")
        print("="*60)
        print(f"{self.description}\n")
    
    def reset(self):
        """Reset environment and render if needed"""
        obs = self.env.reset()
        if self.render_enabled:
            self.env.render()
        return obs
    
    def step(self, action):
        """Execute a single environment step"""
        obs, reward, done, info = self.env.step(action)
        if self.render_enabled:
            self.env.render()
        time.sleep(self.timestep)
        return obs, reward, done, info
    
    @abstractmethod
    def run(self):
        """Execute the demo. Must be implemented by subclasses."""
        pass

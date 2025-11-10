"""
Square pattern demo implementation
"""

from typing import Optional
import numpy as np
import time

from ..base import OSCDemoBase


class SquarePatternDemo(OSCDemoBase):
    """Demo: Move end-effector in a square pattern"""
    
    @property
    def name(self) -> str:
        return "Square Pattern with OSC Position Control"
    
    @property
    def description(self) -> str:
        return "The robot will move its end-effector in a square pattern.\nThis demonstrates basic position-only OSC control."
    
    @property
    def required_controller_type(self) -> Optional[str]:
        return "OSC_POSITION"
    
    def run(self):
        self.print_header()
        self.reset()
        
        # Define a square pattern in the x-y plane
        square_pattern = [
            ([0.3, 0, 0], 50, "Moving RIGHT"),
            ([0, 0.3, 0], 50, "Moving FORWARD"),
            ([-0.3, 0, 0], 50, "Moving LEFT"),
            ([0, -0.3, 0], 50, "Moving BACKWARD"),
        ]
        
        for direction, steps, description in square_pattern:
            print(f"{description}...")
            action = np.array(direction)
            
            for i in range(steps):
                normalized_action = action / np.linalg.norm(action) * 0.5
                full_action = np.append(normalized_action, 1)  # Add gripper
                self.step(full_action)
        
        print("Square pattern complete!\n")
        time.sleep(1)

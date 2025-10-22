"""
Circle pattern demo implementation
"""

import numpy as np
import time

from ...base import OSCDemoBase


class CirclePatternDemo(OSCDemoBase):
    """Demo: Move in a circular pattern"""
    
    @property
    def name(self) -> str:
        return "Circular Pattern"
    
    @property
    def description(self) -> str:
        return "The robot will move its end-effector in a circle.\nThis demonstrates smooth continuous motion."
    
    def run(self):
        self.print_header()
        self.reset()
        
        print("Drawing a circle...")
        radius = 0.15
        num_points = 200
        
        for i in range(num_points):
            angle = (2 * np.pi * i) / num_points
            
            # Compute the delta to next point on circle
            dx = radius * np.cos(angle + 0.1) - radius * np.cos(angle)
            dy = radius * np.sin(angle + 0.1) - radius * np.sin(angle)
            
            # Scale to action range and add orientation + gripper for pose control
            if self.is_pose_control:
                action = np.array([dx * 10, dy * 10, 0, 0, 0, 0, 1])
            else:
                action = np.array([dx * 10, dy * 10, 0, 1])
            
            self.step(action)
        
        print("Circle complete!\n")
        time.sleep(1)

"""
Pose control demo implementation
"""

from typing import Optional
import numpy as np
import time

from ...base import OSCDemoBase


class PoseControlDemo(OSCDemoBase):
    """Demo: Control both position and orientation"""
    
    @property
    def name(self) -> str:
        return "Pose Control (Position + Orientation)"
    
    @property
    def description(self) -> str:
        return "The robot will move while also rotating its end-effector.\nThis demonstrates full 6-DOF OSC control."
    
    @property
    def required_controller_type(self) -> Optional[str]:
        return "OSC_POSE"
    
    def run(self):
        self.print_header()
        self.reset()
        
        print("Moving UP while ROTATING...")
        for i in range(100):
            # Action: [dx, dy, dz, d_roll, d_pitch, d_yaw, gripper]
            action = np.array([0, 0, 0.3, 0, 0, np.sin(i * 0.1) * 0.5, 1])
            self.step(action)
        
        print("Movement complete!\n")
        time.sleep(1)

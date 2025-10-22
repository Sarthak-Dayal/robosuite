"""
Variable impedance demo implementation
"""

from typing import Optional
import numpy as np
import time

from ...base import OSCDemoBase


class VariableImpedanceDemo(OSCDemoBase):
    """Demo: Demonstrate variable impedance control"""
    
    @property
    def name(self) -> str:
        return "Variable Impedance Control"
    
    @property
    def description(self) -> str:
        return "This demonstrates how different stiffness values affect motion.\nNote: This requires OSC with 'variable_kp' impedance mode."
    
    @property
    def required_impedance_mode(self) -> Optional[str]:
        return "variable_kp"
    
    def run(self):
        self.print_header()
        self.reset()
        
        target_motion = np.array([0.3, 0.0, 0.0])
        
        # Low stiffness
        print("Moving with LOW stiffness (soft, compliant)...")
        kp_low = np.array([50, 50, 50, 50, 50, 50])
        for i in range(50):
            action = np.concatenate([kp_low / 300, target_motion * 0.3, [1]])
            self.step(action)
        
        self.reset()
        
        # High stiffness
        print("Moving with HIGH stiffness (stiff, precise)...")
        kp_high = np.array([250, 250, 250, 250, 250, 250])
        for i in range(50):
            action = np.concatenate([kp_high / 300, target_motion * 0.3, [1]])
            self.step(action)
        
        print("Impedance demo complete!\n")
        time.sleep(1)

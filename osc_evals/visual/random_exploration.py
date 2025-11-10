"""
Random exploration demo implementation
"""

import numpy as np
import time

from ..base import OSCDemoBase


class RandomExplorationDemo(OSCDemoBase):
    """Demo: Random actions to explore workspace"""
    
    @property
    def name(self) -> str:
        return "Random Exploration"
    
    @property
    def description(self) -> str:
        return "The robot will perform random movements to explore its workspace."
    
    def run(self):
        self.print_header()
        self.reset()
        
        print("Performing random movements...")
        for i in range(200):
            if self.is_pose_control:
                action = np.append(np.random.uniform(-0.5, 0.5, 6), 1)
            else:
                action = np.append(np.random.uniform(-0.5, 0.5, 3), 1)
            
            self.step(action)
            
            if i % 50 == 0:
                print(f"  Step {i}/200")
        
        print("Random exploration complete!\n")
        time.sleep(1)

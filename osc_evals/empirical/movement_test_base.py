"""
Base class for movement-based empirical tests
"""

from abc import ABC, abstractmethod
from typing import Iterator, Tuple, Optional
import numpy as np
from ..base import OSCDemoBase


class MovementTestBase(OSCDemoBase, ABC):
    """Base class for empirical tests that involve movement patterns"""
    
    @abstractmethod
    def get_movement_pattern(self) -> Iterator[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """
        Generate movement pattern as iterator of (target_position, target_orientation) tuples
        
        Yields:
            - target_position: np.ndarray of shape (3,) - desired x, y, z position
            - target_orientation: np.ndarray of shape (3, 3) or None - desired rotation matrix
                (only for OSC_POSE, None for OSC_POSITION)
        """
        pass
    
    @abstractmethod
    def get_hold_duration(self) -> int:
        """
        Get the number of timesteps to hold at each target position
        
        Returns:
            Number of timesteps to hold
        """
        pass
    
    def get_ee_state(self):
        """
        Get current end-effector position and orientation
        
        Returns:
            Tuple of (position, orientation) as numpy arrays
        """
        robot = self.env.robots[0]
        ee_positions = robot._hand_pos
        ee_orientations = robot._hand_orn
        
        # Get the position and orientation for the first arm
        arm_name = robot.arms[0]
        ee_pos = ee_positions[arm_name]
        ee_ori = ee_orientations[arm_name]
        
        return ee_pos, ee_ori
    
    def get_desired_ee_position(self):
        """
        Get the desired end-effector position from controller
        
        Returns:
            Desired position as numpy array or None
        """
        if hasattr(self.arm_controller, 'goal_pos') and self.arm_controller.goal_pos is not None:
            return self.arm_controller.goal_pos.copy()
        return None
    
    def get_desired_ee_orientation(self):
        """
        Get the desired end-effector orientation from controller
        
        Returns:
            Desired orientation as numpy array or None
        """
        if hasattr(self.arm_controller, 'goal_ori') and self.arm_controller.goal_ori is not None:
            return self.arm_controller.goal_ori.copy()
        return None


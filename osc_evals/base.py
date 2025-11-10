"""
Base class for OSC demonstrations
"""

from abc import ABC, abstractmethod
from typing import Optional
import time
import numpy as np


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
        
        # Initialize controller by triggering an update to set origin frame
        # This is necessary for robots with dynamic bases (like GR1)
        robot = self.env.robots[0]
        if hasattr(robot, 'composite_controller'):
            try:
                print("DEBUG: Attempting to call update_state()...")
                print(f"DEBUG: robot.arms = {robot.arms}")
                print(f"DEBUG: robot.part_controllers keys = {list(robot.part_controllers.keys())}")
                # Debug: Check what values are returned from get_controller_base_pose
                for arm in robot.arms:
                    print(f"DEBUG: Processing arm: {arm}, arm in part_controllers: {arm in robot.part_controllers}")
                    if arm in robot.part_controllers:
                        controller = robot.part_controllers[arm]
                        print(f"DEBUG: Checking {arm} controller")
                        print(f"DEBUG: naming_prefix = {controller.naming_prefix}")
                        print(f"DEBUG: part_name = {controller.part_name}")
                        site_name = f"{controller.naming_prefix}{controller.part_name}_center"
                        print(f"DEBUG: Looking for site: {site_name}")
                        try:
                            site_id = robot.sim.model.site_name2id(site_name)
                            print(f"DEBUG: Found site_id = {site_id}")
                        except Exception as e:
                            print(f"DEBUG: Site not found: {e}")
                        
                        try:
                            base_pos, base_ori = robot.composite_controller.get_controller_base_pose(controller_name=arm)
                            print(f"DEBUG: For {arm}, base_pos = {base_pos}, base_ori shape = {base_ori.shape}")
                            if np.any(np.isnan(base_pos)) or np.any(np.isinf(base_pos)):
                                print(f"DEBUG: base_pos contains NaN/inf!")
                            if np.any(np.isnan(base_ori)) or np.any(np.isinf(base_ori)):
                                print(f"DEBUG: base_ori contains NaN/inf!")
                        except Exception as e:
                            print(f"DEBUG: get_controller_base_pose failed for {arm}: {e}")
                
                robot.composite_controller.update_state()
                print("DEBUG: update_state() succeeded")
                
                # Check if origin was actually set
                for arm in robot.arms:
                    if arm in robot.part_controllers:
                        controller = robot.part_controllers[arm]
                        print(f"DEBUG: After update_state(), {arm} origin_pos = {controller.origin_pos}")
                        print(f"DEBUG: After update_state(), {arm} origin_ori = {controller.origin_ori}")
                        print(f"DEBUG: After update_state(), {arm} has update_origin method: {hasattr(controller, 'update_origin')}")
                        
                        # Manually call update_origin to test
                        base_pos, base_ori = robot.composite_controller.get_controller_base_pose(controller_name=arm)
                        print(f"DEBUG: Manually calling update_origin with base_pos={base_pos}")
                        print(f"DEBUG: Controller object ID: {id(controller)}")
                        controller.update_origin(base_pos, base_ori)
                        print(f"DEBUG: After manual update_origin(), {arm} origin_pos = {controller.origin_pos}")
                        print(f"DEBUG: origin_pos object ID: {id(controller.origin_pos)}")
            except Exception as e:
                # If update_state fails (e.g., sites don't exist), manually set origin from ref frame
                print(f"DEBUG: update_state failed: {e}")
                print("Manually setting origin frame from current ref position")
                for arm in robot.arms:
                    if arm in robot.part_controllers:
                        controller = robot.part_controllers[arm]
                        # Set origin to current ref position and orientation
                        if hasattr(controller, 'ref_pos') and hasattr(controller, 'ref_ori_mat'):
                            print(f"DEBUG: Setting origin for {arm} controller")
                            print(f"DEBUG: ref_pos = {controller.ref_pos}")
                            print(f"DEBUG: ref_ori_mat shape = {controller.ref_ori_mat.shape}")
                            if controller.origin_pos is None:
                                controller.origin_pos = controller.ref_pos.copy()
                                controller.origin_ori = controller.ref_ori_mat.copy()
                                print(f"DEBUG: Set origin_pos = {controller.origin_pos}")
                                print(f"DEBUG: Set origin_ori shape = {controller.origin_ori.shape}")
                            else:
                                print(f"DEBUG: origin_pos already set to {controller.origin_pos}")
        
        if self.render_enabled and hasattr(self.env, 'viewer') and self.env.viewer is not None:
            self.env.render()
        return obs
    
    def step(self, action):
        """Execute a single environment step"""
        # If action is for a single arm but robot has multiple parts, construct full action vector
        robot = self.env.robots[0]
        if hasattr(robot, 'action_dim') and len(action) < robot.action_dim:
            # Construct action dict for composite controllers
            if hasattr(robot, 'create_action_vector'):
                # Determine the action dimensions for each part
                # The arm controller expects a specific number of dimensions
                # (6 for OSC_POSE, 3 for OSC_POSITION, etc.)
                arm_control_dim = self.arm_controller.control_dim
                
                # Split action: arm_action is first 'control_dim' elements, gripper is last element
                arm_action = action[:arm_control_dim]
                gripper_action = action[-1]  # Last element (gripper value, not array)
                
                # Create action dictionary
                action_dict = {}
                action_dict[self.arm_name] = arm_action
                
                # Add gripper action if gripper exists
                if hasattr(robot, 'gripper') and self.arm_name in robot.gripper:
                    gripper = robot.gripper[self.arm_name]
                    if gripper.dof > 1:
                        # Multi-finger gripper - broadcast the value
                        action_dict[f"{self.arm_name}_gripper"] = np.ones(gripper.dof) * gripper_action
                    else:
                        action_dict[f"{self.arm_name}_gripper"] = np.array([gripper_action])
                else:
                    action_dict[f"{self.arm_name}_gripper"] = np.array([gripper_action])
                
                # Add zero actions for disabled or non-existent parts
                # This ensures all required parts have actions
                action = robot.create_action_vector(action_dict)
        
        obs, reward, done, info = self.env.step(action)
        if self.render_enabled and hasattr(self.env, 'viewer') and self.env.viewer is not None:
            self.env.render()
        time.sleep(self.timestep)
        return obs, reward, done, info
    
    @abstractmethod
    def run(self):
        """Execute the demo. Must be implemented by subclasses."""
        pass

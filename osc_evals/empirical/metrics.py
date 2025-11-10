"""
Metrics calculation for OSC controller tracking performance
"""

import numpy as np
from typing import List, Dict, Tuple


class TrackingMetrics:
    """Calculates tracking metrics for OSC controller evaluation"""
    
    def __init__(self):
        self.errors: List[np.ndarray] = []
        self.desired_positions: List[np.ndarray] = []
        self.actual_positions: List[np.ndarray] = []
        self.orientation_errors: List[float] = []
        self.desired_orientations: List[np.ndarray] = []
        self.actual_orientations: List[np.ndarray] = []
        self.time_steps: List[int] = []
    
    def add_observation(self, desired_pos: np.ndarray, actual_pos: np.ndarray,
                       desired_ori: np.ndarray = None, actual_ori: np.ndarray = None,
                       step: int = None):
        """Add a new observation for metrics calculation"""
        error = np.linalg.norm(actual_pos - desired_pos)
        self.errors.append(error)
        self.desired_positions.append(desired_pos.copy())
        self.actual_positions.append(actual_pos.copy())
        self.time_steps.append(step if step is not None else len(self.errors) - 1)
        
        if desired_ori is not None and actual_ori is not None:
            # Calculate orientation error as axis-angle magnitude
            # Using Rodrigues' rotation formula
            R_rel = actual_ori @ desired_ori.T
            trace = np.trace(R_rel)
            angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
            self.orientation_errors.append(angle)
            self.desired_orientations.append(desired_ori.copy())
            self.actual_orientations.append(actual_ori.copy())
    
    def reset(self):
        """Reset all accumulated data"""
        self.errors.clear()
        self.desired_positions.clear()
        self.actual_positions.clear()
        self.orientation_errors.clear()
        self.desired_orientations.clear()
        self.actual_orientations.clear()
        self.time_steps.clear()
    
    def compute_position_metrics(self) -> Dict[str, float]:
        """Compute position tracking metrics"""
        if not self.errors:
            return {}
        
        errors_array = np.array(self.errors)
        pos_errors_array = np.array([
            np.array(actual) - np.array(desired)
            for actual, desired in zip(self.actual_positions, self.desired_positions)
        ])
        
        return {
            'l2_error_mean': float(np.mean(errors_array)),
            'l2_error_max': float(np.max(errors_array)),
            'l2_error_std': float(np.std(errors_array)),
            'x_error_mean': float(np.mean(np.abs(pos_errors_array[:, 0]))),
            'y_error_mean': float(np.mean(np.abs(pos_errors_array[:, 1]))),
            'z_error_mean': float(np.mean(np.abs(pos_errors_array[:, 2]))),
            'x_error_max': float(np.max(np.abs(pos_errors_array[:, 0]))),
            'y_error_max': float(np.max(np.abs(pos_errors_array[:, 1]))),
            'z_error_max': float(np.max(np.abs(pos_errors_array[:, 2]))),
        }
    
    def compute_orientation_metrics(self) -> Dict[str, float]:
        """Compute orientation tracking metrics (if available)"""
        if not self.orientation_errors:
            return {}
        
        ori_errors_array = np.array(self.orientation_errors)
        
        return {
            'orientation_error_mean': float(np.mean(ori_errors_array)),
            'orientation_error_max': float(np.max(ori_errors_array)),
            'orientation_error_std': float(np.std(ori_errors_array)),
        }
    
    def compute_steady_state_error(self, start_idx: int = None) -> float:
        """Compute steady-state error (mean error during specified window)
        
        Args:
            start_idx: Index to start computing from (default: last 50% of data)
        """
        if not self.errors:
            return 0.0
        
        if start_idx is None:
            start_idx = len(self.errors) // 2
        
        steady_state_errors = self.errors[start_idx:]
        return float(np.mean(steady_state_errors))
    
    def compute_settling_time(self, threshold: float = 0.05) -> int:
        """Compute settling time (consecutive steps within threshold)
        
        Args:
            threshold: Fraction of target (e.g., 0.05 = 5% threshold)
        """
        if not self.desired_positions or not self.actual_positions:
            return 0
        
        # Get the target distance for threshold
        initial_dist = np.linalg.norm(self.desired_positions[0] - self.actual_positions[0])
        threshold_dist = initial_dist * threshold
        
        # Find last time we were within threshold
        settled = False
        for i in range(len(self.errors) - 1, -1, -1):
            if self.errors[i] <= threshold_dist:
                settled = True
            elif settled:
                return len(self.errors) - i - 1
        
        return len(self.errors) if settled else 0
    
    def compute_overshoot(self) -> float:
        """Compute maximum overshoot percentage"""
        if not self.errors:
            return 0.0
        
        # Get final target distance
        final_dist = self.errors[-1]
        
        # Find max overshoot (error beyond target)
        max_error = np.max(self.errors)
        
        if final_dist == 0:
            return 0.0 if max_error <= 0 else float('inf')
        
        overshoot = (max_error - final_dist) / final_dist * 100
        return float(max(0, overshoot))
    
    def get_all_metrics(self) -> Dict[str, float]:
        """Compute and return all metrics"""
        metrics = {}
        metrics.update(self.compute_position_metrics())
        metrics.update(self.compute_orientation_metrics())
        metrics['steady_state_error'] = self.compute_steady_state_error()
        metrics['settling_time'] = self.compute_settling_time()
        metrics['overshoot'] = self.compute_overshoot()
        return metrics


def compute_segment_metrics(
    desired_trajectory: List[np.ndarray],
    actual_trajectory: List[np.ndarray],
    desired_orientations: List[np.ndarray] = None,
    actual_orientations: List[np.ndarray] = None,
) -> Dict[str, float]:
    """Compute metrics for a single trajectory segment"""
    metrics_calc = TrackingMetrics()
    
    for i, (desired, actual) in enumerate(zip(desired_trajectory, actual_trajectory)):
        desired_ori = desired_orientations[i] if desired_orientations else None
        actual_ori = actual_orientations[i] if actual_orientations else None
        metrics_calc.add_observation(desired, actual, desired_ori, actual_ori, i)
    
    return metrics_calc.get_all_metrics()


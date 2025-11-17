"""
Utilities for running GR1T1 OSC pose controller sweeps.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from robosuite.utils import transform_utils as T

from ..environment import OSCEnvironmentManager
from ..runner import DemoRunner
from .position_hold_test import PositionHoldTest

DEFAULT_TARGET_PATH = Path(__file__).resolve().parent.parent / "ee_targets_2.json"
DEFAULT_ROBOT = "GR1T1RightArmOnly"


def load_pose_targets(json_path: str | Path = DEFAULT_TARGET_PATH) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Load pose targets from JSON and convert to (pos, rot_mat) tuples."""
    json_path = Path(json_path)
    data = json.loads(json_path.read_text())
    targets = []
    for pose in data:
        if len(pose) != 6:
            raise ValueError(f"Expected pose entries of length 6, got {len(pose)}")
        pos = np.array(pose[:3], dtype=float)
        euler = np.array(pose[3:], dtype=float)
        rot = T.euler2mat(euler)
        targets.append((pos, rot))
    return targets


def build_controller_overrides(config: Dict[str, Any]) -> Dict[str, Any]:
    """Map sweep config parameters to controller override dict."""
    overrides: Dict[str, Any] = {}
    if "kp" in config and config["kp"] is not None:
        overrides["kp"] = float(config["kp"])
    if "damping_ratio" in config and config["damping_ratio"] is not None:
        overrides["damping_ratio"] = float(config["damping_ratio"])

    pos_max = config.get("pos_output_max")
    ori_max = config.get("ori_output_max")
    if pos_max is not None or ori_max is not None:
        pos_limit = float(pos_max) if pos_max is not None else 0.05
        ori_limit = float(ori_max) if ori_max is not None else 0.5
        output_max = [pos_limit] * 3 + [ori_limit] * 3
        overrides["output_max"] = output_max
        overrides["output_min"] = [-value for value in output_max]
    return overrides


def default_search_space(metric: str = "overall_position_error") -> Dict[str, Any]:
    """Return a compact wandb sweep config focusing on OSC pose gains."""
    return {
        "method": "bayes",
        "metric": {"name": metric, "goal": "minimize"},
        "parameters": {
            "kp": {"min": 80.0, "max": 250.0},
            "damping_ratio": {"min": 0.4, "max": 2.0},
            "pos_output_max": {"min": 0.03, "max": 0.07},
            "ori_output_max": {"values": [0.3, 0.4, 0.5, 0.6]},
            "hold_duration": {"values": [100, 125, 150]},
        },
    }


def run_gr1t1_pose_trial(
    config: Dict[str, Any],
    wandb_project: str | None = None,
    wandb_entity: str | None = None,
    log_videos: bool = False,
    robot_name: str = DEFAULT_ROBOT,
) -> Dict[str, Any]:
    """
    Execute a single GR1T1 position-hold trial with the provided config.

    Args:
        config: Sweep/sample dictionary. Should include kp / damping ratio / output limits.
        wandb_project: Optional override for wandb project.
        wandb_entity: Optional override for wandb entity.
        log_videos: Whether to enable video logging (defaults False for speed).
        robot_name: Which robot alias to instantiate (defaults to GR1T1RightArmOnly).

    Returns:
        Dictionary of overall tracking metrics from the evaluation.
    """
    controller_overrides = build_controller_overrides(config)
    targets = load_pose_targets()

    env_manager = OSCEnvironmentManager(robot=robot_name, environment="Lift")
    demo_runner = DemoRunner(env_manager)

    # Ensure we always cleanly close the environment even if errors occur
    try:
        results = demo_runner.run_demo(
            demo_class=PositionHoldTest,
            controller_type="OSC_POSE",
            impedance_mode="fixed",
            render=False,
            controller_overrides=controller_overrides,
            use_wandb=True,
            log_videos=log_videos,
            target_poses=targets,
            num_targets=len(targets),
            hold_duration=int(config.get("hold_duration", 125)),
            move_max_steps=int(config.get("move_max_steps", 500)),
            wandb_project=wandb_project,
            wandb_entity=wandb_entity,
            wandb_run_name=config.get("wandb_run_name"),
            wandb_config=config,
        )
    finally:
        demo_runner.cleanup()

    # Provide a single metric alias that wandb sweeps can track easily
    if results:
        results["overall_position_error"] = results.get("l2_error_mean", None)
    return results


__all__ = [
    "load_pose_targets",
    "build_controller_overrides",
    "default_search_space",
    "run_gr1t1_pose_trial",
]


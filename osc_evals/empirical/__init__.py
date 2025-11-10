"""
Empirical evaluations subpackage

Contains empirical/quantitative evaluations that log metrics to wandb and generate
numerical results alongside visualizations.
"""

from .position_hold_test import PositionHoldTest
from .joint_targets_eval import JointTargetsEval
from ..registry import register_demo

# Register empirical tests
register_demo('position_hold', PositionHoldTest)
register_demo('joint_targets', JointTargetsEval)

__all__ = ['PositionHoldTest', 'JointTargetsEval']

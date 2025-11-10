"""
Visual demo implementations

Import all visual demo classes and register them
"""

from .square_pattern import SquarePatternDemo
from .pose_control import PoseControlDemo
from .circle_pattern import CirclePatternDemo
from .variable_impedance import VariableImpedanceDemo
from .random_exploration import RandomExplorationDemo

from ..registry import register_demo


# Register all visual demos
register_demo('square', SquarePatternDemo)
register_demo('pose', PoseControlDemo)
register_demo('circle', CirclePatternDemo)
register_demo('impedance', VariableImpedanceDemo)
register_demo('random', RandomExplorationDemo)


__all__ = [
    'SquarePatternDemo',
    'PoseControlDemo',
    'CirclePatternDemo',
    'VariableImpedanceDemo',
    'RandomExplorationDemo',
]

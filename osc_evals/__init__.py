"""
OSC Evaluation Framework

A modular evaluation framework for OSC controllers with two main categories:
- visual: Interactive visual demonstrations
- empirical: Quantitative evaluations with metrics logging (wandb)
"""

from .base import OSCDemoBase
from .environment import OSCEnvironmentManager
from .runner import DemoRunner
from .registry import DEMO_REGISTRY, register_demo, get_demo
from .cli import main

# Import visual and empirical subpackages
from . import visual
from . import empirical

__all__ = [
    'OSCDemoBase',
    'OSCEnvironmentManager',
    'DemoRunner',
    'DEMO_REGISTRY',
    'register_demo',
    'get_demo',
    'main',
    'visual',
    'empirical',
]

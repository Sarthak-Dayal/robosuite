"""
Demo registry for managing available demos
"""

from typing import Dict, Optional
from .base import OSCDemoBase


# Global demo registry
DEMO_REGISTRY: Dict[str, type[OSCDemoBase]] = {}


def register_demo(name: str, demo_class: type[OSCDemoBase]):
    """
    Register a demo class with a given name
    
    Args:
        name: Demo identifier (used in CLI)
        demo_class: Demo class that inherits from OSCDemoBase
    """
    if not issubclass(demo_class, OSCDemoBase):
        raise TypeError(f"{demo_class} must inherit from OSCDemoBase")
    
    DEMO_REGISTRY[name] = demo_class


def get_demo(name: str) -> Optional[type[OSCDemoBase]]:
    """
    Get a demo class by name
    
    Args:
        name: Demo identifier
        
    Returns:
        Demo class or None if not found
    """
    return DEMO_REGISTRY.get(name)


def list_demos() -> Dict[str, str]:
    """
    Get a dictionary of all available demos with their descriptions
    
    Returns:
        Dict mapping demo names to descriptions
    """
    # Create a temporary instance (with None env) just to get description
    return {
        name: demo_class(None).description.split('\n')[0]
        for name, demo_class in DEMO_REGISTRY.items()
    }

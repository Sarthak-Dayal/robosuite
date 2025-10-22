"""
Operational Space Controller (OSC) Evaluation Framework

Entry point for the OSC evaluation framework.

A modular evaluation framework for OSC controllers with two categories:
- visual: Interactive visual demonstrations
- empirical: Quantitative evaluations with metrics logging

Usage:
    python osc_evals/demo_osc_controller.py --demo <demo_name>
    
Examples:
    python osc_evals/demo_osc_controller.py --demo square        # Square pattern (visual)
    python osc_evals/demo_osc_controller.py --demo pose          # Pose control (visual)
    python osc_evals/demo_osc_controller.py --demo circle        # Circle pattern (visual)
    python osc_evals/demo_osc_controller.py --demo impedance     # Variable impedance (visual)
    
Adding New Visual Demos:
    1. Create a new file in osc_evals/visual/implementations/
    2. Create a class that inherits from OSCDemoBase
    3. Implement required properties and run() method
    4. Register it in osc_evals/visual/implementations/__init__.py
    
Adding New Empirical Evals:
    1. Create a new file in osc_evals/empirical/
    2. Create a class that inherits from OSCDemoBase
    3. Add wandb logging and metrics collection
    4. Register it appropriately
    
See osc_evals/visual/implementations/square_pattern.py for an example.
"""

import sys
import os

# add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# register all visual demo implementations and run main
import osc_evals.visual.implementations
from osc_evals.cli import main


if __name__ == "__main__":
    main()

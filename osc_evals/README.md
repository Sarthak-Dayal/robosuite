# OSC Evaluations - Modular Framework

A clean, modular framework for creating and running OSC (Operational Space Controller) evaluations in robosuite.

## Overview

This framework supports two types of evaluations:
- **Visual**: Interactive demonstrations that showcase controller capabilities visually
- **Empirical**: Quantitative evaluations that log metrics to wandb and generate numerical results

## Project Structure

```
osc_evals/
├── __init__.py                  # Package exports
├── base.py                      # OSCDemoBase abstract class
├── environment.py               # OSCEnvironmentManager
├── runner.py                    # DemoRunner
├── registry.py                  # Demo registration system
├── cli.py                       # CLI interface
├── visual/                      # Visual demonstrations
│   ├── __init__.py
│   └── implementations/         # All visual demo implementations
│       ├── __init__.py          # Auto-registers visual demos
│       ├── square_pattern.py
│       ├── pose_control.py
│       ├── circle_pattern.py
│       ├── variable_impedance.py
│       └── random_exploration.py
├── empirical/                   # Empirical evaluations
│   └── __init__.py              # Future: quantitative evals with wandb logging
└── demo_osc_controller.py       # Entry point
```

## Running Visual Demos

```bash
# Run the square pattern demo
python osc_evals/demo_osc_controller.py --demo square

# Run with different robot
python osc_evals/demo_osc_controller.py --demo circle --robot UR5e

# Run with different controller settings
python osc_evals/demo_osc_controller.py --demo pose --controller OSC_POSE

# See all options
python osc_evals/demo_osc_controller.py --help
```

## Adding a New Visual Demo

### Step 1: Create Your Demo File

Create a new file in `osc_evals/visual/implementations/`, for example `my_demo.py`:

```python
"""
My custom demo implementation
"""

from typing import Optional
import numpy as np
import time

from ...base import OSCDemoBase


class MyCustomDemo(OSCDemoBase):
    """Demo: Description of what your demo does"""
    
    @property
    def name(self) -> str:
        return "My Custom Demo Name"
    
    @property
    def description(self) -> str:
        return "Brief description of your demo.\nCan be multi-line."
    
    @property
    def required_controller_type(self) -> Optional[str]:
        # Return "OSC_POSE", "OSC_POSITION", or None if flexible
        return None
    
    @property
    def required_impedance_mode(self) -> Optional[str]:
        # Return "fixed", "variable", "variable_kp", or None if flexible
        return None
    
    def run(self):
        """Your demo logic goes here"""
        self.print_header()
        self.reset()
        
        print("Running my custom demo...")
        for i in range(100):
            # Create your action
            action = np.array([0.1, 0.0, 0.0, 1])  # [x, y, z, gripper]
            
            # Step the environment
            self.step(action)
        
        print("Demo complete!\n")
```

### Step 2: Register Your Demo

Add it to `osc_evals/visual/implementations/__init__.py`:

```python
from .my_demo import MyCustomDemo

# Add to the registration calls
register_demo('mydemo', MyCustomDemo)
```

### Step 3: Run It!

```bash
python osc_evals/demo_osc_controller.py --demo mydemo
```

## Adding Empirical Evaluations

For empirical evaluations with wandb logging:

1. Create a new file in `osc_evals/empirical/`
2. Inherit from `OSCDemoBase` 
3. Add wandb initialization and logging:
   ```python
   import wandb
   
   class MyEmpiricalEval(OSCDemoBase):
       def run(self):
           wandb.init(project="osc-evals", name=self.name)
           
           # Your evaluation logic with metrics
           for step in range(num_steps):
               # ... run controller ...
               
               # Log metrics
               wandb.log({
                   'step': step,
                   'position_error': error,
                   'success_rate': success,
               })
           
           wandb.finish()
   ```
4. Register and run similarly to visual demos

## Architecture Benefits

### 1. **Separation of Concerns**
- **Environment Management** (`environment.py`): Handles all robosuite environment setup
- **Demo Base** (`base.py`): Provides common functionality for all demos
- **Demo Runner** (`runner.py`): Manages demo execution and lifecycle
- **Registry** (`registry.py`): Handles demo registration and discovery

### 2. **Easy Extension**
- Add new demos without modifying existing code
- Each demo is self-contained in its own file
- Automatic registration system

### 3. **Reusable Components**
- All demos inherit helper methods from `OSCDemoBase`:
  - `reset()` - Reset environment with optional rendering
  - `step(action)` - Execute action with rendering and timing
  - `print_header()` - Pretty print demo information
  - `is_pose_control` - Auto-detect controller type

### 4. **Smart Configuration**
- Demos can declare required controller types
- Framework automatically reconfigures environment if needed
- No manual environment setup required

## Available Base Class Properties

When creating a demo, you have access to:

- `self.env` - The robosuite environment
- `self.render_enabled` - Whether rendering is on
- `self.timestep` - Time delay between steps
- `self.arm_name` - Name of the robot arm
- `self.arm_controller` - Controller object
- `self.is_pose_control` - Boolean for OSC_POSE vs OSC_POSITION

## Example: Complex Demo Pattern

Here's a more complex example showing pattern generation:

```python
class SpiralDemo(OSCDemoBase):
    @property
    def name(self) -> str:
        return "Spiral Pattern"
    
    @property
    def description(self) -> str:
        return "Robot traces an expanding spiral"
    
    def run(self):
        self.print_header()
        self.reset()
        
        print("Drawing spiral...")
        for i in range(300):
            t = i * 0.05
            radius = 0.1 + t * 0.001  # Expanding
            
            dx = radius * np.cos(t)
            dy = radius * np.sin(t)
            dz = t * 0.001  # Slowly move up
            
            if self.is_pose_control:
                action = np.array([dx, dy, dz, 0, 0, 0, 1])
            else:
                action = np.array([dx, dy, dz, 1])
            
            self.step(action)
        
        print("Spiral complete!\n")
```

## Testing Your Demo

Before committing, test your demo with different configurations:

```bash
# Test with position control
python demo_osc_controller.py --demo mydemo --controller OSC_POSITION

# Test with pose control
python demo_osc_controller.py --demo mydemo --controller OSC_POSE

# Test with different robot
python demo_osc_controller.py --demo mydemo --robot UR5e

# Test without rendering (faster)
python demo_osc_controller.py --demo mydemo --no-render
```

## Tips

1. **Keep demos focused**: Each demo should showcase one concept
2. **Use helper methods**: Leverage `self.step()`, `self.reset()`, etc.
3. **Check controller type**: Use `self.is_pose_control` when action dimensions differ
4. **Add descriptions**: Clear `name` and `description` properties help users
5. **Declare requirements**: Use `required_controller_type` and `required_impedance_mode`

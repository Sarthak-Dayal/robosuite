# OSC Evaluations - Structure Overview

## New Directory Structure

```
osc_evals/
├── __init__.py                          # Main package exports
├── base.py                              # OSCDemoBase - base class for all evals
├── environment.py                       # OSCEnvironmentManager - env setup
├── runner.py                            # DemoRunner - execution manager
├── registry.py                          # Demo registration system
├── cli.py                               # CLI interface
├── README.md                            # Documentation
│
├── visual/                              # Visual/Interactive Demos
│   ├── __init__.py
│   └── implementations/
│       ├── __init__.py                  # Registers all visual demos
│       ├── square_pattern.py
│       ├── pose_control.py
│       ├── circle_pattern.py
│       ├── variable_impedance.py
│       └── random_exploration.py
│
├── empirical/                           # Quantitative Evaluations
│   └── __init__.py                      # Future: wandb-logged evals
│
└── demo_osc_controller.py               # Entry point script
```

## Two Types of Evaluations

### 1. Visual Demos (`osc_evals/visual/`)
- **Purpose**: Interactive demonstrations that showcase controller capabilities
- **Output**: Real-time visualization in MuJoCo viewer
- **Use cases**: 
  - Understanding controller behavior
  - Debugging control issues
  - Creating demos for presentations
  - Quick testing of new control strategies

### 2. Empirical Evals (`osc_evals/empirical/`)
- **Purpose**: Quantitative evaluations with metrics and logging
- **Output**: Numerical results logged to wandb + optional visualizations
- **Use cases**:
  - Benchmarking controller performance
  - Comparing different controller configurations
  - Collecting training data
  - Systematic evaluation with metrics (success rate, error, time, etc.)

## Running Examples

### Visual Demo
```bash
python osc_evals/demo_osc_controller.py --demo square --env PickPlaceCan
```

### Empirical Eval (future)
```bash
python osc_evals/demo_osc_controller.py --eval trajectory_tracking --env PickPlaceCan --log-wandb
```

## Key Benefits

1. **Clear Separation**: Visual vs Empirical evaluations are organizationally separated
2. **Extensible**: Easy to add new demos in either category
3. **Reusable**: Shared base classes and infrastructure
4. **Professional**: Structure suitable for research and production use

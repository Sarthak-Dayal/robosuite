"""
Main CLI interface for OSC evaluations
"""

import argparse
from .environment import OSCEnvironmentManager
from .runner import DemoRunner
from .registry import DEMO_REGISTRY


def main():
    """Main entry point for OSC evaluations"""
    parser = argparse.ArgumentParser(
        description="OSC Controller Evaluations - Visual & Empirical",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Available Demos:
{chr(10).join(f"  {name:12s} - {demo_class(None).description.split(chr(10))[0]}" 
              for name, demo_class in DEMO_REGISTRY.items())}

Examples:
  python demo_osc_controller.py --demo square
  python demo_osc_controller.py --demo circle --robot UR5e
  python demo_osc_controller.py --demo impedance --controller OSC_POSE
        """
    )
    
    parser.add_argument(
        "--demo",
        type=str,
        default="square",
        choices=list(DEMO_REGISTRY.keys()),
        help="Which demo to run"
    )
    parser.add_argument(
        "--controller",
        type=str,
        default="OSC_POSE",
        choices=["OSC_POSE", "OSC_POSITION"],
        help="Controller type to use (may be overridden by demo requirements)"
    )
    parser.add_argument(
        "--impedance",
        type=str,
        default="fixed",
        choices=["fixed", "variable", "variable_kp"],
        help="Impedance mode (may be overridden by demo requirements)"
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="Panda",
        help="Robot to use"
    )
    parser.add_argument(
        "--env",
        type=str,
        default="Lift",
        help="Environment to use"
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Disable rendering"
    )
    
    args = parser.parse_args()
    
    # Print startup info
    print("\n" + "="*60)
    print("ROBOSUITE OSC CONTROLLER DEMO - MODULAR FRAMEWORK")
    print("="*60)
    print(f"Environment: {args.env}")
    print(f"Robot: {args.robot}")
    print(f"Controller: {args.controller}")
    print(f"Impedance Mode: {args.impedance}")
    print(f"Demo: {args.demo}")
    
    # Create environment manager
    env_manager = OSCEnvironmentManager(
        robot=args.robot,
        environment=args.env
    )
    
    # Create initial environment for info display
    env = env_manager.create_environment(
        controller_type=args.controller,
        impedance_mode=args.impedance
    )
    
    env_manager.print_controller_info(env)
    
    # Run demo
    demo_runner = DemoRunner(env_manager)
    
    try:
        demo_class = DEMO_REGISTRY[args.demo]
        demo_runner.run_demo(
            demo_class=demo_class,
            controller_type=args.controller,
            impedance_mode=args.impedance,
            render=not args.no_render
        )
        
        print("\n" + "="*60)
        print("DEMO COMPLETE!")
        print("="*60)
        print(f"\nAvailable demos: {', '.join(DEMO_REGISTRY.keys())}")
        print("For more info, see: osc_controller_explanation.md\n")
        
    finally:
        demo_runner.cleanup()

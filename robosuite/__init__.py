from robosuite.robosuite.environments.base import make

# Manipulation environments
from robosuite.robosuite.environments.manipulation.lift import Lift
from robosuite.robosuite.environments.manipulation.stack import Stack
from robosuite.robosuite.environments.manipulation.nut_assembly import NutAssembly
from robosuite.robosuite.environments.manipulation.pick_place import PickPlace
from robosuite.robosuite.environments.manipulation.door import Door
from robosuite.robosuite.environments.manipulation.wipe import Wipe
from robosuite.robosuite.environments.manipulation.tool_hang import ToolHang
from robosuite.robosuite.environments.manipulation.two_arm_lift import TwoArmLift
from robosuite.robosuite.environments.manipulation.two_arm_peg_in_hole import TwoArmPegInHole
from robosuite.robosuite.environments.manipulation.two_arm_handover import TwoArmHandover
from robosuite.robosuite.environments.manipulation.two_arm_transport import TwoArmTransport

from robosuite.robosuite.environments import ALL_ENVIRONMENTS
from robosuite.robosuite.controllers import (
    ALL_PART_CONTROLLERS,
    load_part_controller_config,
    ALL_COMPOSITE_CONTROLLERS,
    load_composite_controller_config,
)
from robosuite.robosuite.robots import ALL_ROBOTS
from robosuite.robosuite.models.grippers import ALL_GRIPPERS
from robosuite.robosuite.utils.log_utils import ROBOSUITE_DEFAULT_LOGGER

try:
    import robosuite_models
except:
    ROBOSUITE_DEFAULT_LOGGER.warning(
        "Could not import robosuite_models. Some robots may not be available. "
        "If you want to use these robots, please install robosuite_models from "
        "source (https://github.com/ARISE-Initiative/robosuite_models) or through pip install."
    )

try:
    from robosuite.robosuite.examples.third_party_controller.mink_controller import WholeBodyMinkIK

except:
    ROBOSUITE_DEFAULT_LOGGER.warning(
        "Could not load the mink-based whole-body IK. Make sure you install related import properly, otherwise you will not be able to use the default IK controller setting for GR1 robot."
    )

__version__ = "1.5.1"
__logo__ = """
      ;     /        ,--.
     ["]   ["]  ,<  |__**|
    /[_]\  [~]\/    |//  |
     ] [   OOO      /o|__|
"""

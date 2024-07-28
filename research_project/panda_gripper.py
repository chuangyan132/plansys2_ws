import sys
import re
import os

import carb
import numpy as np
from pathlib import Path
from omni.isaac.kit import SimulationApp

FRANKA_STAGE_PATH = "/Franka"
FRANKA_USD_PATH = "/Isaac/Robots/Franka/franka.usd"
CAMERA_PRIM_PATH = f"{FRANKA_STAGE_PATH}/panda_hand/geometry/realsense/realsense_camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
GRAPH_PATH = "/ActionGraph"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    nucleus,
    prims,
    rotations,
    stage,
    viewports,
)
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf, UsdGeom  # noqa E402
import omni.graph.core as og  # noqa E402
import omni
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator


# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)


DynamicCuboid(
    prim_path="/World/blue_cube", # The prim path of the cube in the USD stage
    name="blue_cube", # The unique name used to retrieve the object from the scene later on
    position=np.array([0.3, 0.3, 0.05526]), # Using the current stage units which is in meters by default.
    scale=np.array([0.05, 0.05, 0.05]), # most arguments accept mainly numpy arrays.
    color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
    )


try:
    ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
    ros_domain_id = 0
except KeyError:
    print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
    ros_domain_id = 0


# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                (
                    "SubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("SubscribeGripperCommand", "omni.isaac.ros2_bridge.ROS2SubscribeString"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("OnTick", "omni.graph.action.OnTick"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", ros_domain_id),
                # Setting the /Franka target prim to Articulation Controller node
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ],
        },
    )
except Exception as e:
    print(e)


# Setting the /Franka target prim to Publish JointState node
set_target_prims(
    primPath="/ActionGraph/PublishJointState", targetPrimPaths=[FRANKA_STAGE_PATH]
)

# set_targets(
#     prim=stage.get_current_stage().GetPrimAtPath(GRAPH_PATH + "/setCamera"),
#     attribute="inputs:cameraPrim",
#     target_prim_paths=[CAMERA_PRIM_PATH],
# )

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

# Dock the second camera window
viewport = omni.ui.Workspace.get_window("Viewport")
# rs_viewport = omni.ui.Workspace.get_window(REALSENSE_VIEWPORT_NAME)
# rs_viewport.dock_in(viewport, omni.ui.DockPosition.RIGHT)


while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()
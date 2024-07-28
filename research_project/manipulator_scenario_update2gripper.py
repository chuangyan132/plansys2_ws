import sys
import re
import os

import carb
import numpy as np
from pathlib import Path
from omni.isaac.kit import SimulationApp




FRANKA_STAGE_PATH = "/World/Franka"
FRANKA_USD_PATH = "/Isaac/Robots/Franka/franka.usd"
CAMERA_PRIM_PATH = f"{FRANKA_STAGE_PATH}/panda_hand/geometry/realsense/realsense_camera"
END_EFFECTOR_PRIM_PATH = f"{FRANKA_STAGE_PATH}/panda_rightfinger"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
GRAPH_PATH = "/ActionGraph"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"
GRIPPER_DOF_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]


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
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation


# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)


# Set the robot's position and orientation
rotation_quat = rotations.euler_angles_to_quat([0, 0, 0])  # No rotation


# Adding cubes
DynamicCuboid(
    prim_path="/World/blue_cube",
    name="blue_cube",
    position=np.array([0.3, 0.3, 0.05526]),
    scale=np.array([0.05, 0.05, 0.05]),
    color=np.array([0, 0, 1.0]),
)

DynamicCuboid(
    prim_path="/World/red_cube",
    name="red_cube",
    position=np.array([-0.3, -0.3, 0.05526]),
    scale=np.array([0.05, 0.05, 0.05]),
    color=np.array([1.0, 0, 0]),
)

prims.create_prim(
    FRANKA_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0.01601]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)),
    usd_path=assets_root_path + FRANKA_USD_PATH,
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
                    "SubscribeGripperCommand",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                (
                    "ArticulationControllerGripper",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("OnTick", "omni.graph.action.OnTick"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeGripperCommand.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationControllerGripper.inputs:execIn",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeGripperCommand.inputs:context"),
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
                    "SubscribeGripperCommand.outputs:jointNames",
                    "ArticulationControllerGripper.inputs:jointNames",
                ),
                (
                    "SubscribeGripperCommand.outputs:positionCommand",
                    "ArticulationControllerGripper.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeGripperCommand.outputs:velocityCommand",
                    "ArticulationControllerGripper.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeGripperCommand.outputs:effortCommand",
                    "ArticulationControllerGripper.inputs:effortCommand",
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
                ("ArticulationControllerGripper.inputs:usePath", True),
                ("ArticulationControllerGripper.inputs:robotPath", FRANKA_STAGE_PATH),
                ("SubscribeGripperCommand.inputs:topicName", "isaac_gripper_commands"),
            ],
        },
    )
except Exception as e:
    print(e)


# Setting the /Franka target prim to Publish JointState node
set_target_prims(
    primPath="/ActionGraph/PublishJointState", targetPrimPaths=[FRANKA_STAGE_PATH]
)


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
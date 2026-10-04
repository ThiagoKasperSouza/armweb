"""
Single launch file for the whole headless stack.

Starts:
  robot_state_publisher   -> URDF -> /tf, /tf_static  (needs no GUI)
  arm_sim_node            -> /joint_states at 50 Hz
  foxglove_bridge         -> ws://0.0.0.0:8765 for the browser

Optionally exports the robot to OpenUSD before starting.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _current_prefix_path():
    """Return the AMENT_PREFIX_PATH so `ros2 run` can resolve our package."""
    return os.environ.get("AMENT_PREFIX_PATH", "")


def _robot_description(context):
    """Read the URDF file and return its *contents*.

    robot_state_publisher expects the `robot_description` parameter to hold
    the XML itself. Passing a filesystem path makes it parse an empty
    document ("Error document empty").
    """
    path = LaunchConfiguration("urdf").perform(context)
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _resolved_urdf():
    """The URDF the stack is currently using (prepared one, else the demo)."""
    for cand in ("/ws/assets/urdf/robot.urdf", "/ws/assets/urdf/demo_arm.urdf"):
        if os.path.isfile(cand):
            return cand
    return "/ws/assets/urdf/demo_arm.urdf"


def _sim_env(extra=None):
    env = {"AMENT_PREFIX_PATH": _current_prefix_path()}
    if extra:
        env.update(extra)
    return env


def _launch_nodes(context, *args, **kwargs):
    return None


def _rsp_params(context):
    return [{
        "robot_description": _robot_description(context),
        "publish_frequency": 50.0,
    }]


def _launch_rsp(context, *args, **kwargs):
    """OpaqueFunction wrapper: builds the RSP node with the URDF contents."""
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[_rsp_params(context)[0]],
            remappings=[("/robot_description", "/arm/robot_description")],
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory("armweb_sim")
    # Default to whatever prepare_arm.sh produced; falls back to the demo arm.
    urdf = "/ws/assets/urdf/robot.urdf"
    if not os.path.isfile(urdf):
        urdf = "/ws/assets/urdf/demo_arm.urdf"
    usd_out = LaunchConfiguration("usd_out")

    declare_args = [
        DeclareLaunchArgument("urdf", default_value=urdf),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("usd_out", default_value="/ws/data/usd/arm.usda"),
        DeclareLaunchArgument("export_usd", default_value="true"),
    ]

    export_usd = ExecuteProcess(
        cmd=[
            "python3", "/ws/tools/export_usd.py",
            "--urdf", LaunchConfiguration("urdf"),
            "--out", usd_out,
            "--name", "arm",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("export_usd")),
    )

    # The URDF is plain XML. robot_state_publisher needs the XML *contents* in the
    # `robot_description` parameter, so we read the file at launch time via an
    # OpaqueFunction (this ROS version rejects a callable for `parameters`).
    rsp = OpaqueFunction(function=_launch_rsp)

    # NOTE: ament_python installs console scripts into <prefix>/bin, which
    # `ros2 run` does not search (it looks in libexec). Invoking the module
    # directly with the overlay sourced is the reliable route.
    sim = ExecuteProcess(
        cmd=["python3", "-m", "armweb_sim.arm_sim_node"],
        output="screen",
        additional_env=_sim_env({"ARMWEB_URDF": _resolved_urdf()}),
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("foxglove_port"),
            "address": "0.0.0.0",
            "capabilities": ["clientPublish", "parameters"],
        }],
    )

    # Give the USD export a moment to finish before the rest comes up.
    return LaunchDescription(
        declare_args
        + [
            export_usd,
            TimerAction(period=2.0, actions=[rsp, sim, foxglove]),
        ]
    )
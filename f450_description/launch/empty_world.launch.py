import os
import tempfile
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_urdf(context, *args, **kwargs):
    pkg_share = get_package_share_directory("f450_description")
    robot_xacro = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")

    output_urdf = os.path.join(
        tempfile.gettempdir(),
        "f450_robot.urdf"
    )

    subprocess.run(
        ["ros2", "run", "xacro", "xacro", robot_xacro, "-o", output_urdf],
        check=True,
    )

    return []


def generate_launch_description():
    pkg_share = get_package_share_directory("f450_description")

    world_file = os.path.join(
        pkg_share,
        "worlds",
        "empty_fortress.sdf",
    )

    robot_xacro = os.path.join(
        pkg_share,
        "urdf",
        "robot.urdf.xacro",
    )

    output_urdf = os.path.join(
        tempfile.gettempdir(),
        "f450_robot.urdf",
    )

    resource_path = os.path.abspath(
        os.path.join(pkg_share, "..")
    )

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ":",
            resource_path,
        ],
    )

    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro]),
        value_type=str,
    )

    gazebo = ExecuteProcess(
        cmd=[
            "ign",
            "gazebo",
            world_file,
            "-r",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="f450_uav",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            "/world/empty/create",
            "--reqtype",
            "ignition.msgs.EntityFactory",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{output_urdf}", '
            f'name: "f450_uav", '
            f'pose {{ position {{ x: 0.0 y: 0.0 z: 0.5 }} }}',
        ],
        output="screen",
    )

    delayed_spawn_robot = TimerAction(
        period=2.0,
        actions=[spawn_robot],
    )

    camera_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/f450/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/f450/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    imu_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="imu_bridge",
        arguments=[
            "/f450/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        ],
        output="screen",
    )

    return LaunchDescription([
        ign_resource_path,
        OpaqueFunction(function=generate_urdf),
        gazebo,
        robot_state_publisher,
        delayed_spawn_robot,
        camera_bridge,
        imu_bridge,
    ])
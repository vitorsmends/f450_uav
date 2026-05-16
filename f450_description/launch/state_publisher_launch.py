import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_share = get_package_share_directory("f450_description")

    robot_xacro = os.path.join(
        pkg_share,
        "urdf",
        "robot.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="f450_uav",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        GroupAction([
            PushRosNamespace(namespace),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                }],
                output="screen",
            ),
        ]),
    ])
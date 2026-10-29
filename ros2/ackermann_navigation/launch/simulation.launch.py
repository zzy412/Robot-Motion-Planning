
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = "ackermann_navigation"

    pkg_share = get_package_share_directory(pkg_name)

    rviz_config = os.path.join(
        pkg_share,
        "rviz",
        "ackermann.rviz"
    )

    # Launch arguments
    use_rviz = LaunchConfiguration("use_rviz")
    use_controller = LaunchConfiguration("use_controller")
    speed = LaunchConfiguration("speed")
    lookahead = LaunchConfiguration("lookahead")

    # Hybrid A* planner
    planner = Node(
        package=pkg_name,
        executable="planner_node",
        name="ackermann_planner",
        output="screen"
    )

    # Ackermann vehicle simulator
    vehicle_sim = Node(
        package=pkg_name,
        executable="vehicle_sim_node",
        name="vehicle_sim_node",
        output="screen"
    )

    # Pure Pursuit controller
    controller = Node(
        package=pkg_name,
        executable="controller_node",
        name="controller_node",
        output="screen",
        condition=IfCondition(use_controller),
        parameters=[{
            "wheelbase": 1.2,
            "lookahead": lookahead,
            "speed": speed,
            "max_steer": 0.5235987756,
            "goal_tolerance": 0.6
        }]
    )

    # RViz2 visualization
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2"
        ),

        DeclareLaunchArgument(
            "use_controller",
            default_value="true",
            description="Start Pure Pursuit controller"
        ),

        DeclareLaunchArgument(
            "speed",
            default_value="0.5",
            description="Vehicle target speed in m/s"
        ),

        DeclareLaunchArgument(
            "lookahead",
            default_value="1.5",
            description="Pure Pursuit lookahead distance in m"
        ),

        planner,
        vehicle_sim,
        controller,
        rviz
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from arrc_launch_utils.substitutions import IfElseSubstitution
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
from arrc_launch_utils.utils import RewrittenYaml
import os


def generate_launch_description():
    # Launch arguments
    uav_name = EnvironmentVariable("UAV_NAME")
    uav_name_param = LaunchConfiguration("uav_name", default=uav_name)
    is_sim = EnvironmentVariable("RUN_TYPE")

    use_sim_time = IfElseSubstitution(
        IfCondition(PythonExpression(["'", is_sim, "' == ", "'simulation'"])),
        LaunchConfiguration("use_sim_time", default="true"),
        LaunchConfiguration("use_sim_time", default="false"),
    )

    parsed_config_file = RewrittenYaml(
        source_file=os.path.join(
            get_package_share_directory("m4_navigation_bringup"),
            "config",
            "m4_config.yaml",
        ),
        param_rewrites={
            "use_sim_time": use_sim_time,
        },
        root_key=uav_name_param,
        convert_types=True,
    )

    return LaunchDescription(
        [
            # Pure pursuit controller
            Node(
                package="m4_navigation_tracker",
                executable="m4_navigation_tracker",
                name="m4_navigation_tracker",
                namespace=uav_name_param,
                parameters=[
                    parsed_config_file,
                ],
            ),
            # Morph controller
            Node(
                package="m4_navigation_morph",
                executable="m4_navigation_morph",
                name="m4_navigation_morph",
                namespace=uav_name_param,
                parameters=[
                    parsed_config_file,
                ],
            ),
            # Waypoints GPS
            Node(
                package="m4_navigation_waypoints",
                executable="m4_navigation_waypoints",
                name="m4_navigation_waypoints",
                namespace=uav_name_param,
                parameters=[
                    parsed_config_file,
                ],
            ),
        ]
    )

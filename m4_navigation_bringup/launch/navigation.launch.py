from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from arrc_launch_utils.utils import RewrittenYaml
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    uav_name = EnvironmentVariable("UAV_NAME")
    uav_name_param = LaunchConfiguration("uav_name", default=uav_name)

    parsed_config_file = RewrittenYaml(
        source_file=os.path.join(
            get_package_share_directory("m4_navigation_bringup"),
            "config",
            "m4_config.yaml",
        ),
        param_rewrites={},
        root_key=uav_name_param,
        convert_types=True,
    )

    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
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
        ]
    )

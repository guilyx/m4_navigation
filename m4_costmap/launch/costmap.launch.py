from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("m4_costmap")
    config_file = os.path.join(pkg_dir, "config", "costmap_params.yaml")

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=config_file,
        description="Path to config file for costmap parameters",
    )

    costmap_node = Node(
        package="m4_costmap",
        executable="costmap_node",
        name="costmap",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
        remappings=[
            ("pointcloud", "/livox/lidar"),
        ],
    )

    return LaunchDescription([declare_config_file, costmap_node])

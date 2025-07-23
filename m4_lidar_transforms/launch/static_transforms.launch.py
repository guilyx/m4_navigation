from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from arrc_launch_utils.substitutions import IfElseSubstitution
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get environment variables
    uav_name = EnvironmentVariable("UAV_NAME")
    uav_name_param = LaunchConfiguration("uav_name", default=uav_name)
    is_sim = EnvironmentVariable("RUN_TYPE")

    # Handle simulation time
    use_sim_time = IfElseSubstitution(
        IfCondition(PythonExpression(["'", is_sim, "' == ", "'simulation'"])),
        LaunchConfiguration("use_sim_time", default="true"),
        LaunchConfiguration("use_sim_time", default="false"),
    )

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "parent_frame",
                default_value="base_link",
                description="Parent frame for the Livox LiDAR base link",
            ),
            DeclareLaunchArgument(
                "livox_base_x",
                default_value="0.0",
                description="X offset of Livox base link relative to parent frame",
            ),
            DeclareLaunchArgument(
                "livox_base_y",
                default_value="0.15",
                description="Y offset of Livox base link relative to parent frame",
            ),
            DeclareLaunchArgument(
                "livox_base_z",
                default_value="0.15",
                description="Z offset of Livox base link relative to parent frame",
            ),
            DeclareLaunchArgument(
                "livox_base_roll",
                default_value="0.0",
                description="Roll angle of Livox base link relative to parent frame",
            ),
            DeclareLaunchArgument(
                "livox_base_pitch",
                default_value="0.0",
                description="Pitch angle of Livox base link relative to parent frame",
            ),
            DeclareLaunchArgument(
                "livox_base_yaw",
                default_value="0.0",
                description="Yaw angle of Livox base link relative to parent frame",
            ),
            # Static transform publisher node
            Node(
                package="m4_lidar_transforms",
                executable="static_transform_publisher",
                name="livox_transform_publisher",
                namespace=uav_name_param,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "parent_frame": PathJoinSubstitution(
                            [uav_name_param, LaunchConfiguration("parent_frame")]
                        ),
                        "livox_base_x": LaunchConfiguration("livox_base_x"),
                        "livox_base_y": LaunchConfiguration("livox_base_y"),
                        "livox_base_z": LaunchConfiguration("livox_base_z"),
                        "livox_base_roll": LaunchConfiguration("livox_base_roll"),
                        "livox_base_pitch": LaunchConfiguration("livox_base_pitch"),
                        "livox_base_yaw": LaunchConfiguration("livox_base_yaw"),
                    }
                ],
                output="screen",
            ),
        ]
    )

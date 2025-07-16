from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            # Pure pursuit controller
            Node(
                package="m4_navigation_tracker",
                executable="pure_pursuit_controller",
                name="pure_pursuit_controller",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "lookahead_distance": 2.0,
                        "max_linear_velocity": 0.5,
                        "max_angular_velocity": 1.0,
                        "min_lookahead_distance": 1.0,
                        "max_lookahead_distance": 3.0,
                        "position_tolerance": 0.2,
                        "orientation_tolerance": 0.13,
                    }
                ],
            ),
            # Morph controller
            Node(
                package="m4_navigation_morph",
                executable="morph_controller",
                name="morph_controller",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "morph_duration": 5.0,
                        "morph_angular_velocity": 0.3,
                    }
                ],
            ),
        ]
    )

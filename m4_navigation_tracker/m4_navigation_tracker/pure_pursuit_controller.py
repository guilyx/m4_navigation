
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from m4_msgs.action import TrackPath
from m4_msgs.srv import GpsToPose
import math
import numpy as np
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from rclpy.executors import MultiThreadedExecutor
import tf2_geometry_msgs
from enum import Enum
import time


class ControllerState(Enum):
    IDLE = "idle"
    TRACKING = "tracking"
    ROTATING = "rotating"
    COMPLETED = "completed"


class PurePursuitController(Node):
    def __init__(self):
        super().__init__("pure_pursuit_controller")

        # Parameters
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("lookahead_distance", 1.0)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("xy_goal_tolerance", 0.8)
        self.declare_parameter("yaw_goal_tolerance", 0.2)
        self.declare_parameter("min_angular_velocity", 0.2)

        # Control rate in seconds
        self.control_period = 0.1

        ns = self.get_namespace()
        if ns == "/":
            self.robot_frame = self.get_parameter("robot_frame").value
        else:
            self.robot_frame = (ns + "/" + self.get_parameter("robot_frame").value)[1:]

        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.xy_goal_tolerance = self.get_parameter("xy_goal_tolerance").value
        self.yaw_goal_tolerance = self.get_parameter("yaw_goal_tolerance").value
        self.min_angular_velocity = self.get_parameter("min_angular_velocity").value

        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance}")
        self.get_logger().info(f"Max linear velocity: {self.max_linear_velocity}")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_velocity}")
        self.get_logger().info(f"XY goal tolerance: {self.xy_goal_tolerance}")
        self.get_logger().info(f"Yaw goal tolerance: {self.yaw_goal_tolerance}")
        self.get_logger().info(f"Robot frame: {self.robot_frame}")
        self.get_logger().info(f"Control period: {self.control_period}")
        self.get_logger().info(f"Min angular velocity: {self.min_angular_velocity}")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Debug visualization publishers
        self.path_marker_pub = self.create_publisher(
            MarkerArray, "debug/path_markers", 10
        )
        self.lookahead_marker_pub = self.create_publisher(
            Marker, "debug/lookahead_circle", 10
        )

        # Service client for GPS to pose conversion
        self.gps_to_pose_client = self.create_client(GpsToPose, "gps_to_pose")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action server
        self._action_server = ActionServer(
            self,
            TrackPath,
            "track_path",
            self.execute_tracking_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # State variables
        self.state: ControllerState = ControllerState.IDLE
        self.path: list[PoseStamped] | None = None
        self.current_goal_idx: int = 0
        self.last_status_log_time: float = 0.0  # Add this line

        # Create timer for transform monitoring (2 Hz)
        # self.create_timer(0.5, self.monitor_transform_callback)

        self.get_logger().info("Pure Pursuit Controller initialized")

    # def monitor_transform_callback(self):
    #     """Timer callback to monitor transform between robot and GPS origin frame"""
    #     try:
    #         transform = self.tf_buffer.lookup_transform(
    #             self.robot_frame,
    #             "Drone1/gps_origin",
    #             rclpy.time.Time(),
    #             rclpy.duration.Duration(seconds=1.0),
    #         )

    #         # Extract position and yaw
    #         x = transform.transform.translation.x
    #         y = transform.transform.translation.y
    #         z = transform.transform.translation.z

    #         # Get yaw from quaternion
    #         qz = transform.transform.rotation.z
    #         qw = transform.transform.rotation.w
    #         yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

    #         self.get_logger().info(
    #             f"Transform robot->gps_origin: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={math.degrees(yaw):.1f}°"
    #         )

    #     except Exception as e:
    #         self.get_logger().error(f"Failed to get transform: {str(e)}")

    def convert_gps_to_poses(
        self, poses: list[PoseStamped]
    ) -> list[PoseStamped] | None:
        """Convert GPS coordinates in PoseStamped messages to local poses

        Args:
            poses (list[PoseStamped]): List of PoseStamped messages containing GPS coordinates

        Returns:
            list[PoseStamped] | None: List of converted PoseStamped messages, or None if conversion fails
        """
        if not self.gps_to_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GPS to pose service not available")
            return None

        converted_poses: list[PoseStamped] = []
        for pose in poses:
            request: GpsToPose.Request = GpsToPose.Request()
            request.latitude = pose.pose.position.x  # Using x as latitude
            request.longitude = pose.pose.position.y  # Using y as longitude
            request.altitude = pose.pose.position.z
            ns = self.get_namespace()
            request.frame_id = ns + "/gps_origin"

            try:
                response: GpsToPose.Response = self.gps_to_pose_client.call(request)
                if not response.success:
                    self.get_logger().error("Failed to convert GPS point")
                    return None
                converted_poses.append(response.pose)
            except Exception as e:
                self.get_logger().error(f"Failed to convert GPS point: {str(e)}")
                return None

        return converted_poses

    def project_point_on_horizon(
        self, target_x: float, target_y: float
    ) -> tuple[float, float]:
        """Project a point onto the lookahead horizon circle

        This method takes coordinates from a Pose object's position and projects them onto
        the lookahead horizon circle if they are beyond the lookahead distance.

        Args:
            target_x (float): Target x coordinate from Pose.position.x
            target_y (float): Target y coordinate from Pose.position.y

        Returns:
            tuple[float, float]: Projected x and y coordinates on the horizon circle,
                               suitable for creating a new Pose.position
        """
        # Calculate distance to target
        distance: float = math.sqrt(target_x**2 + target_y**2)

        if distance <= self.lookahead_distance:
            return target_x, target_y

        # Calculate the scaling factor to project onto horizon circle
        scale: float = self.lookahead_distance / distance

        # Project the point onto the horizon circle
        projected_x: float = target_x * scale
        projected_y: float = target_y * scale

        return projected_x, projected_y

    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle (rotation around Z axis)

        Args:
            qx (float): Quaternion x component
            qy (float): Quaternion y component
            qz (float): Quaternion z component
            qw (float): Quaternion w component

        Returns:
            float: Yaw angle in radians
        """
        # Handle edge cases to avoid numerical instability
        if abs(qw) < 1e-6 and abs(qz) < 1e-6:
            return 0.0

        # Extract yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw: float) -> tuple[float, float, float, float]:
        """Convert yaw angle to quaternion

        Args:
            yaw (float): Yaw angle in radians

        Returns:
            tuple[float, float, float, float]: Quaternion components (x, y, z, w)
        """
        # For pure yaw rotation, x and y components are 0
        qx = 0.0
        qy = 0.0
        # z and w components represent rotation in the x-y plane
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def transform_pose_to_robot_frame(self, pose_stamped: PoseStamped) -> Pose | None:
        """Transform a pose to the robot frame

        Args:
            pose_stamped (PoseStamped): The pose to transform

        Returns:
            Pose | None: The transformed pose in robot frame, or None if transformation fails
        """
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.robot_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0),
            )

            # Extract yaw from pose and transform
            pose_yaw = self.quaternion_to_yaw(
                0.0,
                0.0,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w,
            )
            transform_yaw = self.quaternion_to_yaw(
                0.0, 0.0, transform.transform.rotation.z, transform.transform.rotation.w
            )

            # Combine rotations
            combined_yaw = pose_yaw + transform_yaw
            qx, qy, qz, qw = self.yaw_to_quaternion(combined_yaw)

            transformed_pose = Pose()
            # Translation: add vectors
            transformed_pose.position.x = (
                pose_stamped.pose.position.x + transform.transform.translation.x
            )
            transformed_pose.position.y = (
                pose_stamped.pose.position.y + transform.transform.translation.y
            )
            transformed_pose.position.z = (
                pose_stamped.pose.position.z + transform.transform.translation.z
            )
            # Set combined rotation
            transformed_pose.orientation.x = 0.0
            transformed_pose.orientation.y = 0.0
            transformed_pose.orientation.z = qz
            transformed_pose.orientation.w = qw

            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose: {str(e)}")
            return None

    def find_lookahead_point(self, pose: Pose) -> Pose:
        """Calculate the lookahead point from a pose in robot frame

        Args:
            pose (Pose): The pose in robot frame to calculate lookahead from

        Returns:
            Pose: The lookahead point, either the original pose or projected onto horizon circle
        """
        # Calculate distance to target
        distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

        # If we're within lookahead distance, return the pose as is
        if distance <= self.lookahead_distance:
            return pose

        # Project onto horizon circle
        projected_x, projected_y = self.project_point_on_horizon(
            pose.position.x, pose.position.y
        )

        # Create projected pose
        projected_pose = Pose()
        projected_pose.position.x = projected_x
        projected_pose.position.y = projected_y
        projected_pose.position.z = pose.position.z

        # Calculate orientation towards the actual target
        angle = math.atan2(pose.position.y, pose.position.x)
        projected_pose.orientation.z = math.sin(angle / 2.0)
        projected_pose.orientation.w = math.cos(angle / 2.0)

        return projected_pose

    def move(
        self, target_x: float, target_y: float, target_yaw: float | None = None
    ) -> Twist:
        """
        Calculate control commands based on target position and orientation.

        Args:
            target_x (float): Target x position in robot frame
            target_y (float): Target y position in robot frame
            target_yaw (float | None): Target yaw in robot frame

        Returns:
            Twist: Control commands for the robot
        """
        cmd_vel: Twist = Twist()

        # If we're in rotation mode, only handle orientation
        if target_yaw is not None and self.state == ControllerState.ROTATING:
            # Use proportional control for final orientation
            # Current yaw is 0 in robot frame
            angle_diff: float = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
            cmd_vel.angular.z = min(
                self.max_angular_velocity,
                max(-self.max_angular_velocity, 2.0 * angle_diff),
            )
            if abs(cmd_vel.angular.z) < self.min_angular_velocity:
                cmd_vel.angular.z = math.copysign(
                    self.min_angular_velocity, cmd_vel.angular.z
                )

            self.get_logger().info(
                f"Target yaw: {target_yaw}, Angular velocity: {cmd_vel.angular.z}"
            )
            return cmd_vel

        # Calculate distance to target point
        distance: float = math.sqrt(target_x**2 + target_y**2)

        # Constants for control
        DISTANCE_EPSILON = 0.01  # Minimum distance threshold for curvature calculation
        DISTANCE_SCALE_THRESHOLD = (
            self.lookahead_distance
        )  # Distance at which to start scaling velocity
        MIN_LINEAR_VELOCITY = (
            0.3 * self.max_linear_velocity
        )  # Minimum forward velocity when tracking

        # Angle thresholds for blending between rotation and tracking
        FULL_ROTATION_THRESHOLD = math.pi * 0.8  # ~145 degrees - full rotation mode
        START_ROTATION_THRESHOLD = (
            math.pi * 0.4
        )  # ~72 degrees - start blending rotation

        # Calculate angle to goal (this represents our heading error)
        angle_to_goal = math.atan2(target_y, target_x)
        abs_angle = abs(angle_to_goal)

        # Calculate rotation and tracking blend factors
        if abs_angle >= FULL_ROTATION_THRESHOLD:
            # Pure rotation mode
            rotation_factor = 1.0
            tracking_factor = 0.0
        elif abs_angle <= START_ROTATION_THRESHOLD:
            # Pure tracking mode
            rotation_factor = 0.0
            tracking_factor = 1.0
        else:
            # Blend between rotation and tracking
            blend_range = FULL_ROTATION_THRESHOLD - START_ROTATION_THRESHOLD
            blend_factor = (abs_angle - START_ROTATION_THRESHOLD) / blend_range
            # Smooth the transition using a sine function
            blend_factor = math.sin(blend_factor * math.pi / 2)
            rotation_factor = blend_factor
            tracking_factor = 1.0 - blend_factor

        # Calculate rotation command
        rotation_direction = 1.0 if angle_to_goal > 0 else -1.0
        rotation_command = (
            rotation_direction * self.max_angular_velocity * rotation_factor
        )

        # Calculate tracking command
        if tracking_factor > 0:
            # Base velocity calculation
            base_velocity = self.max_linear_velocity

            # Heading factor only affects excess speed above minimum
            heading_factor = math.cos(
                angle_to_goal
            )  # 1.0 when aligned, < 0 when behind
            heading_factor = max(
                0.0, heading_factor
            )  # Don't let heading reduce speed below base

            # Calculate final velocity
            # Ensure we maintain good forward speed when tracking
            linear_velocity = (
                MIN_LINEAR_VELOCITY
                + (base_velocity - MIN_LINEAR_VELOCITY) * heading_factor
            ) * tracking_factor

            # Pure pursuit control law
            curvature: float = (
                2.0 * target_y / (distance**2) if distance > DISTANCE_EPSILON else 0.0
            )

            # Calculate angular velocity for tracking
            tracking_angular = linear_velocity * curvature

            # Apply velocity limits while preserving curvature
            if abs(tracking_angular) > self.max_angular_velocity:
                scale: float = self.max_angular_velocity / abs(tracking_angular)
                tracking_angular = math.copysign(
                    self.max_angular_velocity, tracking_angular
                )
                linear_velocity *= scale

            # Set commands with tracking component
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = -1 * tracking_angular
        else:
            # Pure rotation mode
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = rotation_command

        # Blend final angular velocity if we're in the transition zone
        if 0.0 < rotation_factor < 1.0:
            cmd_vel.angular.z = (
                cmd_vel.angular.z * tracking_factor + rotation_command * rotation_factor
            )

        # Log the control factors
        self.get_logger().debug(
            f"Control: angle={math.degrees(angle_to_goal):.1f}°, "
            f"rot_factor={rotation_factor:.2f}, track_factor={tracking_factor:.2f}, "
            f"cmd_vel=[{cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f}]"
        )

        return cmd_vel

    def execute_tracking_callback(self, goal_handle) -> TrackPath.Result:
        """Action server callback to execute path tracking

        Args:
            goal_handle: The goal handle for this action

        Returns:
            TrackPath.Result: The result of the path tracking action
        """
        self.get_logger().info("Starting path tracking")

        # Log the received goal
        self.path = goal_handle.request.path
        self.get_logger().info(f"Received path with {len(self.path)} waypoints")
        self.last_status_log_time = time.time()  # Initialize the timer

        # Convert GPS coordinates if needed
        if goal_handle.request.is_gps:
            self.get_logger().info("Converting GPS coordinates to local poses")
            self.path = self.convert_gps_to_poses(self.path)
            self.get_logger().info(f"Converted path: {self.path}")
            if not self.path:
                goal_handle.abort()
                return TrackPath.Result(
                    success=False, message="Failed to convert GPS coordinates"
                )

        # Publish initial visualization
        self.publish_path_markers()
        self.publish_lookahead_circle()

        self.current_goal_idx = 0
        self.state = ControllerState.TRACKING
        feedback_msg: TrackPath.Feedback = TrackPath.Feedback()

        while rclpy.ok():
            # Check if action has been canceled
            if goal_handle.is_cancel_requested:
                self.stop_tracking()
                goal_handle.canceled()
                return TrackPath.Result(success=False, message="Path tracking canceled")

            # Update visualization
            self.publish_path_markers()
            self.publish_lookahead_circle()

            try:
                # Get current goal and transform to robot frame
                current_goal = (
                    self.path[self.current_goal_idx]
                    if self.current_goal_idx < len(self.path)
                    else self.path[-1]
                )
                transformed_pose = self.transform_pose_to_robot_frame(current_goal)
                if not transformed_pose:
                    self.stop_tracking()
                    goal_handle.abort()
                    return TrackPath.Result(
                        success=False, message="Failed to transform current goal"
                    )

                # Calculate distance and orientation
                distance = math.sqrt(
                    transformed_pose.position.x**2 + transformed_pose.position.y**2
                )
                qz = transformed_pose.orientation.z
                qw = transformed_pose.orientation.w
                theta = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

                # Throttled logging (every 5 seconds)
                current_time = time.time()
                if current_time - self.last_status_log_time >= 5.0:
                    self.get_logger().info(
                        f"Status: Goal {self.current_goal_idx + 1}/{len(self.path)}, "
                        f"Distance: {distance:.2f}m, "
                        f"Position: [x: {transformed_pose.position.x:.2f}, y: {transformed_pose.position.y:.2f}]"
                    )
                    self.last_status_log_time = current_time

                # Check if we've reached the current goal
                if (
                    distance < self.xy_goal_tolerance
                    or self.state == ControllerState.ROTATING
                ):
                    if self.current_goal_idx == len(self.path) - 1:
                        if self.state != ControllerState.ROTATING:
                            self.state = ControllerState.ROTATING
                            self.get_logger().info("Switching to rotation state")
                        # If we're in rotation state and orientation is achieved, we're done
                        elif (
                            self.state == ControllerState.ROTATING
                            and abs(theta) < self.yaw_goal_tolerance
                        ):
                            self.get_logger().info("Reached final goal")
                            self.state = ControllerState.COMPLETED
                            break

                    else:
                        self.current_goal_idx += 1
                        continue

                lookahead_pose = self.find_lookahead_point(transformed_pose)
                cmd_vel = self.move(
                    lookahead_pose.position.x, lookahead_pose.position.y, theta
                )

                # Publish control commands
                self.cmd_vel_pub.publish(cmd_vel)

                # Update feedback
                feedback_msg.current_velocity = cmd_vel.linear.x
                feedback_msg.current_angular_velocity = cmd_vel.angular.z
                feedback_msg.current_state = self.state.value
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)

            except Exception as e:
                self.get_logger().error(f"Error during path tracking: {str(e)}")
                self.stop_tracking()
                goal_handle.abort()
                return TrackPath.Result(
                    success=False,
                    message=f"Error during path tracking: {str(e)}",
                )

            # Sleep to maintain control rate
            time.sleep(self.control_period)

        self.stop_tracking()
        goal_handle.succeed()
        return TrackPath.Result(
            success=True, message="Successfully completed path tracking"
        )

    def stop_tracking(self):
        """Stop all motion and reset state"""
        self.state = ControllerState.IDLE
        self.path = None
        self.current_goal_idx = 0

        # Stop motion
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().info("Path tracking stopped")

    def publish_path_markers(self) -> None:
        """Publish visualization markers for the path"""
        if not self.path:
            return

        try:
            marker_array = MarkerArray()

            # Path line strip marker
            line_strip = Marker()
            line_strip.header.frame_id = self.robot_frame
            line_strip.header.stamp = self.get_clock().now().to_msg()
            line_strip.ns = "path"
            line_strip.id = 0
            line_strip.type = Marker.LINE_STRIP
            line_strip.action = Marker.ADD
            line_strip.pose.orientation.w = 1.0
            line_strip.scale.x = 0.05  # Line width
            line_strip.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green line

            # Waypoint sphere markers
            waypoints = Marker()
            waypoints.header.frame_id = self.robot_frame
            waypoints.header.stamp = self.get_clock().now().to_msg()
            waypoints.ns = "waypoints"
            waypoints.id = 1
            waypoints.type = Marker.SPHERE_LIST
            waypoints.action = Marker.ADD
            waypoints.pose.orientation.w = 1.0
            waypoints.scale.x = 0.1  # Sphere size
            waypoints.scale.y = 0.1
            waypoints.scale.z = 0.1
            waypoints.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red spheres

            # Transform and add points to both markers
            for pose_stamped in self.path:
                # Transform pose to robot frame
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    pose_stamped.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0),
                )
                transformed_pose = tf2_geometry_msgs.do_transform_pose(
                    pose_stamped.pose, transform
                )

                point = Point()
                point.x = transformed_pose.position.x
                point.y = transformed_pose.position.y
                point.z = transformed_pose.position.z
                line_strip.points.append(point)
                waypoints.points.append(point)

            # Add markers to array
            marker_array.markers.append(line_strip)
            marker_array.markers.append(waypoints)

            # Publish marker array
            self.path_marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f"Failed to publish path markers: {str(e)}")

    def publish_lookahead_circle(self) -> None:
        """Publish visualization marker for the lookahead circle"""
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead_circle"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02  # Line width
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)  # Blue circle

        # Create circle points
        num_points = 32
        for i in range(num_points + 1):
            theta = 2.0 * math.pi * i / num_points
            point = Point()
            point.x = self.lookahead_distance * math.cos(theta)
            point.y = self.lookahead_distance * math.sin(theta)
            point.z = 0.0
            marker.points.append(point)

        # Publish marker
        self.lookahead_marker_pub.publish(marker)


def main(args=None):
    try:
        rclpy.init(args=args)
        controller = PurePursuitController()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(controller)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

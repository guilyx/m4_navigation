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
import tf2_geometry_msgs
from enum import Enum


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
        self.declare_parameter("xy_goal_tolerance", 0.1)
        self.declare_parameter("yaw_goal_tolerance", 0.1)

        self.control_rate = rclpy.duration.Duration(seconds=0.1)

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
        self.tf_broadcaster = TransformBroadcaster(self)

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

        self.get_logger().info("Pure Pursuit Controller initialized")

    def transform_path_to_initial_frame(
        self, path: list[PoseStamped]
    ) -> list[Pose] | None:
        """Transform path to initial robot pose frame

        Args:
            path (list[PoseStamped]): List of PoseStamped messages to transform

        Returns:
            list[Pose] | None: List of transformed Pose messages in robot frame, or None if transformation fails
        """
        if not path:
            return None

        # Store initial robot pose from first goal frame
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.robot_frame,
                path[0].header.frame_id,
                path[0].header.stamp,
                rclpy.duration.Duration(seconds=1.0),
            )

            # Create a frame for initial robot pose
            initial_frame: TransformStamped = TransformStamped()
            initial_frame.header.stamp = self.get_clock().now().to_msg()
            initial_frame.header.frame_id = self.robot_frame
            initial_frame.child_frame_id = "initial_robot_pose"
            initial_frame.transform.translation.x = transform.transform.translation.x
            initial_frame.transform.translation.y = transform.transform.translation.y
            initial_frame.transform.translation.z = transform.transform.translation.z
            initial_frame.transform.rotation = transform.transform.rotation

            # Broadcast the initial robot pose frame
            self.tf_broadcaster.sendTransform(initial_frame)

            # Transform all poses to initial robot frame
            transformed_path: list[Pose] = []
            for pose_stamped in path:
                # Transform the pose - need to pass the pose field from PoseStamped
                transformed_pose: Pose = tf2_geometry_msgs.do_transform_pose(
                    pose_stamped.pose, transform
                )
                transformed_path.append(transformed_pose)

            # Log the transformed path
            self.get_logger().info("Transformed path:")
            for i, pose in enumerate(transformed_path):
                self.get_logger().info(
                    f"Waypoint {i}: x={pose.position.x:.2f}, y={pose.position.y:.2f}, "
                    f"z={pose.position.z:.2f}"
                )

            return transformed_path

        except Exception as e:
            self.get_logger().error(f"Failed to transform path: {str(e)}")
            return None

    def convert_gps_to_poses(
        self, poses: list[PoseStamped]
    ) -> list[PoseStamped] | None:
        """Convert GPS coordinates in PoseStamped messages to local poses

        Args:
            poses (list[PoseStamped]): List of PoseStamped messages containing GPS coordinates

        Returns:
            list[PoseStamped] | None: List of converted PoseStamped messages, or None if conversion fails
        """
        if not self.gps_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GPS to pose service not available")
            return None

        converted_poses: list[PoseStamped] = []
        for pose in poses:
            request: GpsToPose.Request = GpsToPose.Request()
            request.latitude = pose.pose.position.x  # Using x as latitude
            request.longitude = pose.pose.position.y  # Using y as longitude
            request.altitude = pose.pose.position.z
            request.frame_id = pose.header.frame_id

            try:
                response: GpsToPose.Response = self.gps_to_pose_client.call(request)
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
                pose_stamped.header.stamp,
                rclpy.duration.Duration(seconds=1.0),
            )

            # Transform the pose to robot frame
            transformed_pose: Pose = tf2_geometry_msgs.do_transform_pose(
                pose_stamped.pose, transform
            )
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

    def rotate_to_goal(self, goal_pose: PoseStamped) -> bool:
        """Rotate to face the goal orientation

        Args:
            goal_pose (PoseStamped): The goal pose to rotate to

        Returns:
            bool: True if rotation is complete, False otherwise
        """
        # Extract target yaw from quaternion
        qx: float = goal_pose.pose.orientation.x
        qy: float = goal_pose.pose.orientation.y
        qz: float = goal_pose.pose.orientation.z
        qw: float = goal_pose.pose.orientation.w

        # Convert quaternion to Euler angles
        siny_cosp: float = 2.0 * (qw * qz + qx * qy)
        cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
        target_yaw: float = math.atan2(siny_cosp, cosy_cosp)

        # Current yaw is 0 in robot frame
        current_yaw: float = 0.0

        # Calculate the angle difference
        angle_diff: float = math.atan2(
            math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)
        )

        # Check if we've reached the target orientation
        if abs(angle_diff) < self.yaw_goal_tolerance:
            return True

        # Create rotation command
        cmd_vel: Twist = Twist()
        cmd_vel.angular.z = self.max_angular_velocity * angle_diff / math.pi
        self.cmd_vel_pub.publish(cmd_vel)

        return False

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
            return cmd_vel

        # Calculate distance to target point
        distance: float = math.sqrt(target_x**2 + target_y**2)

        # If we're very close to target, slow down
        distance_factor: float = min(1.0, distance / self.lookahead_distance)

        # Pure pursuit control law
        # Calculate curvature (k = 2x/L^2, where x is lateral error and L is lookahead distance)
        # For robot frame, lateral error is target_y
        curvature: float = 2.0 * target_y / (distance**2) if distance > 0.01 else 0.0

        # Calculate target velocities
        # Forward velocity scales with distance to target and inversely with curvature
        linear_velocity: float = self.max_linear_velocity * distance_factor
        # Angular velocity is proportional to curvature and forward velocity
        angular_velocity: float = linear_velocity * curvature

        # Apply velocity limits
        if abs(angular_velocity) > self.max_angular_velocity:
            # Scale down both velocities to maintain the same curvature
            scale: float = self.max_angular_velocity / abs(angular_velocity)
            angular_velocity = math.copysign(
                self.max_angular_velocity, angular_velocity
            )
            linear_velocity *= scale

        # Ensure minimum turning radius
        min_radius: float = 0.1  # minimum turning radius in meters
        if abs(linear_velocity) > abs(angular_velocity) * min_radius:
            linear_velocity = abs(angular_velocity) * min_radius

        # Set commands
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

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
        self.get_logger().info(f"Received path with {len(self.path)} waypoints:")
        for i, pose in enumerate(self.path):
            self.get_logger().info(
                f"Goal {i}: frame_id={pose.header.frame_id}, "
                f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, "
                f"z={pose.pose.position.z:.2f}"
            )

        # Convert GPS coordinates if needed
        if goal_handle.request.is_gps:
            self.get_logger().info("Converting GPS coordinates to local poses")
            self.path = self.convert_gps_to_poses(self.path)
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

            if self.state == ControllerState.TRACKING:
                # Update visualization
                self.publish_path_markers()
                self.publish_lookahead_circle()

                # Get current goal and transform to robot frame
                if self.current_goal_idx >= len(self.path):
                    self.state = ControllerState.ROTATING
                    continue

                current_goal = self.path[self.current_goal_idx]
                transformed_pose = self.transform_pose_to_robot_frame(current_goal)
                if not transformed_pose:
                    self.stop_tracking()
                    goal_handle.abort()
                    return TrackPath.Result(
                        success=False, message="Failed to transform current goal"
                    )

                # Log the transformed goal
                distance = math.sqrt(
                    transformed_pose.position.x**2 + transformed_pose.position.y**2
                )
                self.get_logger().info(
                    f"Current goal {self.current_goal_idx} (in {self.robot_frame} frame): "
                    f"x={transformed_pose.position.x:.2f}, y={transformed_pose.position.y:.2f}, "
                    f"distance={distance:.2f}"
                )

                # Check if we've reached the current goal
                if distance < self.xy_goal_tolerance:
                    if self.current_goal_idx == len(self.path) - 1:
                        self.get_logger().info(
                            "Reached final waypoint, transitioning to rotation"
                        )
                        self.state = ControllerState.ROTATING
                        continue
                    self.current_goal_idx += 1
                    continue

                # Find lookahead point
                lookahead_pose = self.find_lookahead_point(transformed_pose)

                # Log the lookahead point
                lookahead_distance = math.sqrt(
                    lookahead_pose.position.x**2 + lookahead_pose.position.y**2
                )
                self.get_logger().info(
                    f"Lookahead point (in {self.robot_frame} frame): "
                    f"x={lookahead_pose.position.x:.2f}, y={lookahead_pose.position.y:.2f}, "
                    f"distance={lookahead_distance:.2f}"
                )

                # Calculate and publish control commands
                cmd_vel: Twist = self.move(
                    lookahead_pose.position.x, lookahead_pose.position.y
                )
                self.cmd_vel_pub.publish(cmd_vel)

                # Update feedback
                feedback_msg.current_velocity = cmd_vel.linear.x
                feedback_msg.current_angular_velocity = cmd_vel.angular.z
                feedback_msg.current_state = self.state.value
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)

            elif self.state == ControllerState.ROTATING:
                # Update visualization
                self.publish_path_markers()
                self.publish_lookahead_circle()

                try:
                    # Transform final pose to robot frame
                    final_goal = self.path[-1]
                    final_pose = self.transform_pose_to_robot_frame(final_goal)
                    if not final_pose:
                        raise Exception("Failed to transform final pose")

                    # Extract target yaw from final pose quaternion
                    qx: float = final_pose.orientation.x
                    qy: float = final_pose.orientation.y
                    qz: float = final_pose.orientation.z
                    qw: float = final_pose.orientation.w

                    # Convert quaternion to Euler angles
                    siny_cosp: float = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
                    target_yaw: float = math.atan2(siny_cosp, cosy_cosp)

                    # Calculate and publish control commands
                    cmd_vel: Twist = self.move(0.0, 0.0, target_yaw)
                    self.cmd_vel_pub.publish(cmd_vel)

                    # Check if we've reached the target orientation
                    if abs(cmd_vel.angular.z) < self.yaw_goal_tolerance:
                        self.state = ControllerState.COMPLETED
                        break

                    # Update feedback
                    feedback_msg.current_velocity = 0.0
                    feedback_msg.current_angular_velocity = cmd_vel.angular.z
                    feedback_msg.current_state = self.state.value
                    feedback_msg.distance_remaining = 0.0
                    goal_handle.publish_feedback(feedback_msg)

                except Exception as e:
                    self.get_logger().error(
                        f"Failed to handle final rotation: {str(e)}"
                    )
                    self.stop_tracking()
                    goal_handle.abort()
                    return TrackPath.Result(
                        success=False,
                        message=f"Failed to handle final rotation: {str(e)}",
                    )

            # Sleep to maintain control rate
            self.get_clock().sleep_for(self.control_rate)

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
                    pose_stamped.header.stamp,
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
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
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

        self.robot_frame = self.get_parameter("robot_frame").value
        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.xy_goal_tolerance = self.get_parameter("xy_goal_tolerance").value
        self.yaw_goal_tolerance = self.get_parameter("yaw_goal_tolerance").value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

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
        self.state = ControllerState.IDLE
        self.initial_robot_pose = None
        self.transformed_path = None
        self.current_goal_idx = 0

        self.get_logger().info("Pure Pursuit Controller initialized")

    def transform_path_to_initial_frame(self, path):
        """Transform path to initial robot pose frame"""
        if not path:
            return None

        # Store initial robot pose from first goal frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                path[0].header.frame_id,
                path[0].header.stamp,
                rclpy.duration.Duration(seconds=1.0),
            )

            # Create a frame for initial robot pose
            initial_frame = TransformStamped()
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
            transformed_path = []
            for pose in path:
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                transformed_path.append(transformed_pose)

            return transformed_path

        except Exception as e:
            self.get_logger().error(f"Failed to transform path: {str(e)}")
            return None

    def convert_gps_to_poses(self, poses):
        """Convert GPS coordinates in PoseStamped messages to local poses"""
        if not self.gps_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GPS to pose service not available")
            return None

        converted_poses = []
        for pose in poses:
            request = GpsToPose.Request()
            request.latitude = pose.pose.position.x  # Using x as latitude
            request.longitude = pose.pose.position.y  # Using y as longitude
            request.altitude = pose.pose.position.z
            request.frame_id = pose.header.frame_id

            try:
                response = self.gps_to_pose_client.call(request)
                converted_poses.append(response.pose)
            except Exception as e:
                self.get_logger().error(f"Failed to convert GPS point: {str(e)}")
                return None

        return converted_poses

    def project_point_on_horizon(self, target_x, target_y):
        """Project a point onto the lookahead horizon circle"""
        # Calculate distance to target
        distance = math.sqrt(target_x**2 + target_y**2)

        if distance <= self.lookahead_distance:
            return target_x, target_y

        # Calculate the scaling factor to project onto horizon circle
        scale = self.lookahead_distance / distance

        # Project the point onto the horizon circle
        projected_x = target_x * scale
        projected_y = target_y * scale

        return projected_x, projected_y

    def find_lookahead_point(self):
        """Find the lookahead point in robot frame"""
        if not self.transformed_path or self.current_goal_idx >= len(
            self.transformed_path
        ):
            return None

        # Current position is always 0,0 in robot frame
        robot_x, robot_y = 0.0, 0.0

        # Get current target point
        target = self.transformed_path[self.current_goal_idx]
        target_x = target.pose.position.x
        target_y = target.pose.position.y

        # Calculate distance to target
        distance = math.sqrt(target_x**2 + target_y**2)

        # If we're close enough to current target, move to next one if available
        if distance < self.xy_goal_tolerance:
            self.current_goal_idx += 1
            if self.current_goal_idx >= len(self.transformed_path):
                return None
            target = self.transformed_path[self.current_goal_idx]
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            distance = math.sqrt(target_x**2 + target_y**2)

        # If target is beyond lookahead distance, project it onto the horizon circle
        if distance > self.lookahead_distance:
            projected_x, projected_y = self.project_point_on_horizon(target_x, target_y)

            # Create a new PoseStamped for the projected point
            projected_pose = PoseStamped()
            projected_pose.header = target.header
            projected_pose.pose.position.x = projected_x
            projected_pose.pose.position.y = projected_y

            # Calculate orientation towards the actual target
            angle = math.atan2(target_y, target_x)
            projected_pose.pose.orientation.z = math.sin(angle / 2.0)
            projected_pose.pose.orientation.w = math.cos(angle / 2.0)

            return projected_pose

        return target

    def rotate_to_goal(self, goal_pose):
        """Rotate to face the goal orientation"""
        # Extract target yaw from quaternion
        qx = goal_pose.pose.orientation.x
        qy = goal_pose.pose.orientation.y
        qz = goal_pose.pose.orientation.z
        qw = goal_pose.pose.orientation.w

        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        target_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Current yaw is 0 in robot frame
        current_yaw = 0.0

        # Calculate the angle difference
        angle_diff = math.atan2(
            math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)
        )

        # Check if we've reached the target orientation
        if abs(angle_diff) < self.yaw_goal_tolerance:
            return True

        # Create rotation command
        cmd_vel = Twist()
        cmd_vel.angular.z = self.max_angular_velocity * angle_diff / math.pi
        self.cmd_vel_pub.publish(cmd_vel)

        return False

    def execute_tracking_callback(self, goal_handle):
        """Action server callback to execute path tracking"""
        self.get_logger().info("Starting path tracking")

        path = goal_handle.request.path

        # Convert GPS coordinates if needed
        if goal_handle.request.is_gps:
            self.get_logger().info("Converting GPS coordinates to local poses")
            path = self.convert_gps_to_poses(path)
            if not path:
                goal_handle.abort()
                return TrackPath.Result(
                    success=False, message="Failed to convert GPS coordinates"
                )

        # Transform path to initial robot frame
        self.transformed_path = self.transform_path_to_initial_frame(path)
        if not self.transformed_path:
            goal_handle.abort()
            return TrackPath.Result(success=False, message="Failed to transform path")

        self.current_goal_idx = 0
        self.state = ControllerState.TRACKING
        feedback_msg = TrackPath.Feedback()

        while rclpy.ok():
            # Check if action has been canceled
            if goal_handle.is_cancel_requested:
                self.stop_tracking()
                goal_handle.canceled()
                return TrackPath.Result(success=False, message="Path tracking canceled")

            if self.state == ControllerState.TRACKING:
                lookahead_point = self.find_lookahead_point()

                if not lookahead_point:
                    if self.current_goal_idx >= len(self.transformed_path):
                        self.state = ControllerState.ROTATING
                    else:
                        self.stop_tracking()
                        goal_handle.abort()
                        return TrackPath.Result(
                            success=False, message="Failed to find lookahead point"
                        )
                    continue

                # Calculate the angle to the lookahead point (already in robot frame)
                target_angle = math.atan2(
                    lookahead_point.pose.position.y, lookahead_point.pose.position.x
                )

                # Calculate velocities
                angular_velocity = self.max_angular_velocity * target_angle / math.pi
                linear_velocity = self.max_linear_velocity * (
                    1 - abs(angular_velocity) / self.max_angular_velocity
                )

                # Create and publish Twist message
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_velocity
                cmd_vel.angular.z = angular_velocity
                self.cmd_vel_pub.publish(cmd_vel)

                # Update feedback
                feedback_msg.current_velocity = linear_velocity
                feedback_msg.current_angular_velocity = angular_velocity
                feedback_msg.current_state = self.state.value
                if self.current_goal_idx < len(self.transformed_path):
                    feedback_msg.distance_remaining = math.sqrt(
                        self.transformed_path[self.current_goal_idx].pose.position.x**2
                        + self.transformed_path[self.current_goal_idx].pose.position.y
                        ** 2
                    )
                goal_handle.publish_feedback(feedback_msg)

            elif self.state == ControllerState.ROTATING:
                # Rotate to final orientation
                if self.rotate_to_goal(self.transformed_path[-1]):
                    self.state = ControllerState.COMPLETED
                    break

                # Update feedback
                feedback_msg.current_velocity = 0.0
                feedback_msg.current_angular_velocity = (
                    self.cmd_vel_pub.get_last_published().angular.z
                )
                feedback_msg.current_state = self.state.value
                feedback_msg.distance_remaining = 0.0
                goal_handle.publish_feedback(feedback_msg)

            # Sleep to maintain control rate
            rclpy.sleep(0.1)

        self.stop_tracking()
        goal_handle.succeed()
        return TrackPath.Result(
            success=True, message="Successfully completed path tracking"
        )

    def stop_tracking(self):
        """Stop all motion and reset state"""
        self.state = ControllerState.IDLE
        self.transformed_path = None
        self.current_goal_idx = 0

        # Stop motion
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().info("Path tracking stopped")


def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

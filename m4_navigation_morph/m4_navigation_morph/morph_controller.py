#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from custom_msgs.msg import TiltVel
from m4_msgs.action import Morph
import time


class MorphController(Node):
    def __init__(self):
        super().__init__("morph_controller")

        # Parameters
        self.declare_parameter("morph_duration", 5.0)  # Duration in seconds
        self.declare_parameter("morph_angular_velocity", 0.3)  # rad/s

        self.morph_duration = self.get_parameter("morph_duration").value
        self.morph_angular_velocity = self.get_parameter("morph_angular_velocity").value
        self.fly_tilt_velocity = 1.0
        self.drive_tilt_velocity = -1.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.tilt_vel_pub = self.create_publisher(TiltVel, "tilt_vel", 10)

        # Action server
        self._action_server = ActionServer(
            self,
            Morph,
            "morph",
            self.execute_morph_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Morph Controller initialized")

    def execute_morph_callback(self, goal_handle):
        """Action server callback to execute morphing"""
        self.get_logger().info(
            f'Starting morph to {"fly" if goal_handle.request.to_fly_mode else "drive"} mode'
        )

        feedback_msg = Morph.Feedback()
        start_time = time.time()

        # Create messages
        cmd_vel = Twist()
        cmd_vel.angular.z = self.morph_angular_velocity

        tilt_vel = TiltVel()
        tilt_vel.value = (
            self.fly_tilt_velocity
            if goal_handle.request.to_fly_mode
            else self.drive_tilt_velocity
        )

        # Morphing loop
        while (time.time() - start_time) < self.morph_duration:
            # Check if action has been canceled
            if goal_handle.is_cancel_requested:
                self.stop_morphing()
                goal_handle.canceled()
                return Morph.Result(success=False, message="Morphing canceled")

            # Publish commands
            self.cmd_vel_pub.publish(cmd_vel)
            self.tilt_vel_pub.publish(tilt_vel)

            # Update feedback
            current_time = time.time()
            elapsed = current_time - start_time
            feedback_msg.time_elapsed = elapsed
            feedback_msg.time_remaining = self.morph_duration - elapsed
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to maintain control rate
            time.sleep(0.1)

        # Stop all motion
        self.stop_morphing()

        # Return success
        goal_handle.succeed()
        return Morph.Result(
            success=True,
            message=f'Successfully morphed to {"fly" if goal_handle.request.to_fly_mode else "drive"} mode',
        )

    def stop_morphing(self):
        """Stop all motion"""
        # Stop rotation
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        # Stop tilting
        tilt_vel = TiltVel()
        tilt_vel.value = 0.0
        self.tilt_vel_pub.publish(tilt_vel)

        self.get_logger().info("Morphing stopped")


def main(args=None):
    rclpy.init(args=args)
    controller = MorphController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class LivoxTransformPublisher : public rclcpp::Node
{
public:
  LivoxTransformPublisher() : Node("livox_transform_publisher")
  {
    // Create static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Declare parameters
    this->declare_parameter("parent_frame", "base_link");
    this->declare_parameter("livox_base_x", 0.0);    // From SDF default
    this->declare_parameter("livox_base_y", 0.15);     // From SDF default
    this->declare_parameter("livox_base_z", 0.15);    // From SDF default
    this->declare_parameter("livox_base_roll", 0.0);  // From SDF default
    this->declare_parameter("livox_base_pitch", 0.0); // From SDF default
    this->declare_parameter("livox_base_yaw", 0.0);   // From SDF default

    // Get parameters
    std::string parent_frame = this->get_parameter("parent_frame").as_string();
    double base_x = this->get_parameter("livox_base_x").as_double();
    double base_y = this->get_parameter("livox_base_y").as_double();
    double base_z = this->get_parameter("livox_base_z").as_double();
    double base_roll = this->get_parameter("livox_base_roll").as_double();
    double base_pitch = this->get_parameter("livox_base_pitch").as_double();
    double base_yaw = this->get_parameter("livox_base_yaw").as_double();

    std::string ns = get_namespace();

    // Create and publish transforms
    publishTransform(parent_frame, ns + "/livox_mid_360_base_link", base_x, base_y, base_z, base_roll, base_pitch, base_yaw);

    // From SDF: livox_frame relative to livox_mid_360_base_link
    publishTransform(ns + "/livox_mid_360_base_link", ns + "/livox_frame", 0.0, 0.0, 0.0327, // x, y, z
                     0.0, 0.0, -3.14159);                                        // roll, pitch, yaw

    // From SDF: livox_frame/lidar relative to livox_frame
    publishTransform(ns + "/livox_frame", ns + "/livox_frame/lidar", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // From SDF: livox_mid_360_imu_link relative to livox_frame
    publishTransform(ns + "/livox_frame", ns + "/livox_mid_360_imu_link", 0.011, 0.02329, -0.04412, 0.0, 0.0, 0.0);

    // From SDF: livox_mid_360_imu_link/imu relative to livox_mid_360_imu_link
    publishTransform(ns + "/livox_mid_360_imu_link", ns + "/livox_mid_360_imu_link/imu", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "Published static transforms for Livox LiDAR frames");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  void publishTransform(const std::string& parent_frame, const std::string& child_frame, double x, double y, double z, double roll,
                        double pitch, double yaw)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    // Convert roll, pitch, yaw to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy;

    tf_static_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}
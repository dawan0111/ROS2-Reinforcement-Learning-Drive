#include "reinforcement_learning_drive/actor/differential_actor.hpp"
#include <iostream>

namespace ReinforcementLearningDrive {

DifferentialActor::DifferentialActor(const rclcpp::Node::SharedPtr& node, std::string actor_name)
    : ROS2Actor(node, actor_name) {
  m_initialize();
}
DifferentialActor::DifferentialActor(const rclcpp::Node::SharedPtr& node, std::string actor_name, double frequency)
    : ROS2Actor(node, actor_name, frequency) {
  m_initialize();
}

void DifferentialActor::m_initialize() {
  m_wheel_base = 0.4;
  m_collision_space = {0.225, -0.225, 0.225, 0.225, -0.225, 0.225, -0.225, -0.225};
}

void DifferentialActor::m_reset() {
  ROS2Actor::m_reset();
}

void DifferentialActor::m_predictPose(const Command& twist, double dt) {
  const auto& pose = getCurrentPose();
  auto new_pose = *pose;

  double wheel_base = m_wheel_base;

  double linear_velocity = twist.linear.x;
  double angular_velocity = twist.angular.z;

  double current_yaw = quatToYaw();

  if (std::abs(angular_velocity) < 1e-5) {
    new_pose.pose.position.x += linear_velocity * dt * std::cos(current_yaw);
    new_pose.pose.position.y += linear_velocity * dt * std::sin(current_yaw);
  } else {
    double turning_radius = linear_velocity / angular_velocity;
    double delta_yaw = angular_velocity * dt;

    new_pose.pose.position.x += turning_radius * (std::sin(current_yaw + delta_yaw) - std::sin(current_yaw));
    new_pose.pose.position.y -= turning_radius * (std::cos(current_yaw + delta_yaw) - std::cos(current_yaw));
    current_yaw += delta_yaw;
  }

  new_pose.pose.orientation = yawToQuat(current_yaw);

  updatePose(std::move(new_pose));
}

}  // namespace ReinforcementLearningDrive

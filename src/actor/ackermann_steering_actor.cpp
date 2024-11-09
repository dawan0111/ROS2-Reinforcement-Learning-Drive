#include "reinforcement_learning_drive/actor/ackermann_steering_actor.hpp"
#include <iostream>

namespace ReinforcementLearningDrive {

AckermannSteeringActor::AckermannSteeringActor(const rclcpp::Node::SharedPtr& node, std::string actor_name)
    : ROS2Actor(node, actor_name) {
  m_initialize();
}
AckermannSteeringActor::AckermannSteeringActor(const rclcpp::Node::SharedPtr& node, std::string actor_name,
                                               double frequency)
    : ROS2Actor(node, actor_name, frequency) {
  m_initialize();
}

void AckermannSteeringActor::m_initialize() {
  m_steering_angle = 0.0;
  m_wheel_base = 0.4;
  m_collision_space = {0.225, -0.15, 0.225, 0.15, -0.225, 0.15, -0.225, -0.15};
}

void AckermannSteeringActor::m_reset() {
  m_steering_angle = 0.0;
  ROS2Actor::m_reset();
}

void AckermannSteeringActor::m_predictPose(const Command& twist, double dt) {
  const auto& pose = getCurrentPose();
  double wheelbase = m_wheel_base;

  double linear_velocity = twist.linear.x;
  double angular_velocity = twist.angular.z;

  double steering_angle = std::atan2(angular_velocity * wheelbase, linear_velocity);
  m_steering_angle = steering_angle;

  double current_x = pose->pose.position.x;
  double current_y = pose->pose.position.y;

  double current_yaw = quatToYaw();

  double turning_radius;
  if (std::abs(steering_angle) < 1e-5) {
    turning_radius = std::numeric_limits<double>::infinity();
  } else {
    turning_radius = wheelbase / std::tan(steering_angle);
  }

  if (std::isinf(turning_radius)) {
    current_x += linear_velocity * dt * std::cos(current_yaw);
    current_y += linear_velocity * dt * std::sin(current_yaw);
  } else {
    double delta_yaw = linear_velocity * dt / turning_radius;  // 회전으로 인한 Yaw 변화량
    current_x += turning_radius * (std::sin(current_yaw + delta_yaw) - std::sin(current_yaw));
    current_y += turning_radius * (std::cos(current_yaw) - std::cos(current_yaw + delta_yaw));
    current_yaw += delta_yaw;
  }

  pose->pose.position.x = current_x;
  pose->pose.position.y = current_y;
  pose->pose.orientation = yawToQuat(current_yaw);
}

}  // namespace ReinforcementLearningDrive

#include "reinforcement_learning_drive/reward/path_tracking_reward.hpp"

namespace ReinforcementLearningDrive {
PathTrackingReward::PathTrackingReward(const rclcpp::Node::SharedPtr& node, bool loop)
    : ROS2Reward(node), m_loop(loop), m_current_index(0) {
  // Path 토픽 구독 설정
  m_path_sub = m_node->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        m_path = *msg;
        m_current_index = 0;
        RCLCPP_INFO(m_node->get_logger(), "Path received with %ld points", m_path.poses.size());
      });
}

double PathTrackingReward::calculateReward() {
  if (m_path.poses.empty()) {
    RCLCPP_WARN(m_node->get_logger(), "No path data available.");
    return 0.0;
  }
  const auto& target_pose = m_path.poses[m_current_index].pose.position;
  double current_x = 0.0;  // 실제 로봇의 위치 데이터를 받아야 함
  double current_y = 0.0;  // 실제 로봇의 위치 데이터를 받아야 함

  double distance_to_target = std::hypot(target_pose.x - current_x, target_pose.y - current_y);

  // 간단한 보상 함수 예시: 목표에 가까울수록 높은 보상
  double reward = 1.0 / (1.0 + distance_to_target);

  if (distance_to_target < 0.1) {
    m_current_index++;
    if (m_current_index >= static_cast<int>(m_path.poses.size())) {
      if (m_loop) {
        m_current_index = 0;
      } else {
        m_current_index = m_path.poses.size() - 1;
      }
    }
  }

  return reward;
}
}  // namespace ReinforcementLearningDrive
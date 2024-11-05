#include "reinforcement_learning_drive/reward/path_tracking_reward.hpp"

namespace ReinforcementLearningDrive {
PathTrackingReward::PathTrackingReward(const rclcpp::Node::SharedPtr& node, bool loop)
    : ROS2Reward(node), m_loop(loop), m_current_index(0) {
  // Path 토픽 구독 설정
  m_path_sub = m_node->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        m_path = *msg;
        m_current_index = 0;
        m_is_initialized = true;
        RCLCPP_INFO(m_node->get_logger(), "Path received with %ld points", m_path.poses.size());
      });
}

bool PathTrackingReward::calculateReward(const std::shared_ptr<Actor>& actor) {
  if (m_path.poses.empty()) {
    return false;
  }

  const auto& actor_pose = actor->getCurrentPose();
  const auto& target_pose = m_path.poses[m_current_index].pose.position;

  double current_x = actor_pose->pose.position.x;
  double current_y = actor_pose->pose.position.y;

  double distance_to_target = std::hypot(target_pose.x - current_x, target_pose.y - current_y);
  double goal_direction = std::atan2(target_pose.y - current_y, target_pose.x - current_x);
  double current_yaw = actor->quatToYaw();

  double reward = 1.0 / (1.001 + distance_to_target);

  if (distance_to_target < 0.1) {
    m_current_index++;
    m_score++;
    if (m_current_index >= static_cast<int>(m_path.poses.size())) {
      if (m_loop) {
        m_current_index = 0;
      } else {
        m_current_index = m_path.poses.size() - 1;
      }
    }
  }

  m_goal_distance = distance_to_target;
  m_goal_angle = goal_direction - current_yaw;
  m_score = round(m_score) + reward;

  return true;
}
}  // namespace ReinforcementLearningDrive
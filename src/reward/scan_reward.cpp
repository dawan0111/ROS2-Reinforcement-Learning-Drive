#include "reinforcement_learning_drive/reward/scan_reward.hpp"
#include <cmath>
#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {

ScanReward::ScanReward(const rclcpp::Node::SharedPtr& node) : ROS2Reward(node) {}

bool ScanReward::calculateReward(const std::shared_ptr<Actor>& actor) {
  if (!m_is_initialized) {
    m_is_initialized = true;
    m_score = 0.0;
  }

  // `actor`로부터 scan_data를 가져옴
  const auto& actor_status = actor->getActorStatus();
  const auto& scan_data = actor_status->scan_data;

  if (scan_data.empty()) {
    return false;
  }

  // 왼쪽과 오른쪽 스캔 데이터 분리 및 초기화
  double left_sum = 0.0, right_sum = 0.0;
  int left_count = 0, right_count = 0;

  // 각 데이터의 각도에 따라 왼쪽과 오른쪽으로 나눔
  for (const auto& [distance, angle] : scan_data) {
    auto real_distance = std::isinf(distance) ? 10 : distance * 10.0;
    if (angle > 0.01) {
      right_sum += real_distance;
      right_count++;
    } else if (angle < 0.01) {
      left_sum += real_distance;
      left_count++;
    }
  }

  double left_avg = (left_count > 0) ? left_sum / left_count : 0.0;
  double right_avg = (right_count > 0) ? right_sum / right_count : 0.0;

  double distance_diff = std::abs(left_avg - right_avg);
  double reward = std::exp(-distance_diff);
  // RCLCPP_INFO(m_node->get_logger(), "Reword: %f", reward);
  m_score = reward;

  return true;
}

void ScanReward::reset() {
  m_score = 0.0;
}

}  // namespace ReinforcementLearningDrive

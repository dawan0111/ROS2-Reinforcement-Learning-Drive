#include "reinforcement_learning_drive/reward/scan_reward.hpp"
#include <cmath>
#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {

ScanReward::ScanReward(const rclcpp::Node::SharedPtr& node) : ROS2Reward(node) {}

bool ScanReward::calculateReward(const std::shared_ptr<Actor>& actor, const std::shared_ptr<EnvStatus>& status) {
  if (!m_is_initialized) {
    m_is_initialized = true;
    m_score = 0.0;
  }

  const auto& scan_data = status->scan_data;

  if (scan_data.empty()) {
    return false;
  }

  double left_sum = 0.0, right_sum = 0.0;
  int left_count = 0, right_count = 0;

  for (const auto& [distance, angle] : scan_data) {
    if (angle < -M_PI_2 || angle > M_PI_2) {
      continue;
    }

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

  m_score = reward;

  return true;
}

void ScanReward::reset() {
  m_score = 0.0;
}

}  // namespace ReinforcementLearningDrive

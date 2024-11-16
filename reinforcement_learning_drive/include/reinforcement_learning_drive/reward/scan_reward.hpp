#ifndef __SCAN_REWARD_H__
#define __SCAN_REWARD_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "reinforcement_learning_drive/reward/ros2_reward.hpp"

namespace ReinforcementLearningDrive {

class Actor;

class ScanReward : public ROS2Reward {
 public:
  ScanReward(const rclcpp::Node::SharedPtr&);
  bool calculateReward(const std::shared_ptr<Actor>& actor, const std::shared_ptr<EnvStatus>& status) override;
  void reset() override;
};

}  // namespace ReinforcementLearningDrive

#endif  // __SCAN_REWARD_H__

#ifndef __ROS2_REWARD_H__
#define __ROS2_REWARD_H__

#include "rclcpp/rclcpp.hpp"
#include "reinforcement_learning_drive/reward/reward.hpp"

namespace ReinforcementLearningDrive {
class ROS2Reward : public Reward {
 public:
  using Ptr = std::shared_ptr<ROS2Reward>;
  ROS2Reward(const rclcpp::Node::SharedPtr& node) : Reward(), m_node(node){};

 protected:
  rclcpp::Node::SharedPtr m_node;
};

}  // namespace ReinforcementLearningDrive
#endif  // __ROS2_REWARD_H__
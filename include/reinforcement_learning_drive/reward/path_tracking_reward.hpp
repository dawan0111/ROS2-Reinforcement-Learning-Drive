#ifndef __PATH_TRACKING_REWARD_H__
#define __PATH_TRACKING_REWARD_H__

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reinforcement_learning_drive/reward/ros2_reward.hpp"

namespace ReinforcementLearningDrive {
class PathTrackingReward : public ROS2Reward {
 public:
  using Ptr = std::shared_ptr<ROS2Reward>;
  PathTrackingReward(const rclcpp::Node::SharedPtr&, bool loop = true);
  double calculateReward() override;

 private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
  nav_msgs::msg::Path m_path;
  bool m_loop;
  int16_t m_current_index{0};
};

}  // namespace ReinforcementLearningDrive
#endif  // __PATH_TRACKING_REWARD_H__
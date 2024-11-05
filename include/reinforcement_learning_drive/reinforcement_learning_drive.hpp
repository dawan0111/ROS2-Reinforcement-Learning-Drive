#ifndef __REINFORCEMENT_LEARNING_DRIVE_H__
#define __REINFORCEMENT_LEARNING_DRIVE_H__

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "reinforcement_learning_drive/actor/ackermann_steering_actor.hpp"
#include "reinforcement_learning_drive/environment/occupancy_grid_environment.hpp"
#include "reinforcement_learning_drive/reward/path_tracking_reward.hpp"

namespace ReinforcementLearningDrive {
class ReinforcementLearningDrive : public rclcpp::Node {
 public:
  explicit ReinforcementLearningDrive(const rclcpp::NodeOptions&);
  void initialize();

 private:
  std::shared_ptr<Actor> m_actor;
  std::shared_ptr<Environment> m_environment;
  std::shared_ptr<Reward> m_reward;
  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  geometry_msgs::msg::Twist m_current_twist;
};
}  // namespace ReinforcementLearningDrive

#endif  // __REINFORCEMENT_LEARNING_DRIVE_H__
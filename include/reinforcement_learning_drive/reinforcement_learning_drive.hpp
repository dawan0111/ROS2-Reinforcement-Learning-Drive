#ifndef __REINFORCEMENT_LEARNING_DRIVE_H__
#define __REINFORCEMENT_LEARNING_DRIVE_H__

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include "geometry_msgs/msg/twist.hpp"
#include "reinforcement_learning_drive/actor/ackermann_steering_actor.hpp"
#include "reinforcement_learning_drive/environment/occupancy_grid_environment.hpp"
#include "reinforcement_learning_drive/reward/path_tracking_reward.hpp"
#include "reinforcement_learning_drive/reward/scan_reward.hpp"
#include "reinforcement_learning_drive_interface/action/drive.hpp"
#include "reinforcement_learning_drive_interface/srv/drive.hpp"

namespace ReinforcementLearningDrive {
class ReinforcementLearningDrive : public rclcpp::Node {
 public:
  using Drive = reinforcement_learning_drive_interface::srv::Drive;
  using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

  explicit ReinforcementLearningDrive(const rclcpp::NodeOptions&);
  void initialize();

 private:
  void executeAction(const std::shared_ptr<GoalHandleDrive> goal_handle);
  void executeService(const std::shared_ptr<Drive::Request> request, std::shared_ptr<Drive::Response> response);

  // rclcpp_action::Server<Drive>::SharedPtr m_drive_action_server;
  std::shared_ptr<rclcpp::Service<Drive>> m_drive_service_server;

  std::shared_ptr<Actor> m_actor;
  std::shared_ptr<Environment> m_environment;
  std::shared_ptr<Reward> m_reward;
  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  geometry_msgs::msg::Twist m_current_twist;

  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
};
}  // namespace ReinforcementLearningDrive

#endif  // __REINFORCEMENT_LEARNING_DRIVE_H__
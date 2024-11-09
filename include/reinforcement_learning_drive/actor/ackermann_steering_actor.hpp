#ifndef __ACKERMANN_STEERING_ACTOR_H__
#define __ACKERMANN_STEERING_ACTOR_H__

#include "reinforcement_learning_drive/actor/ros2_actor.hpp"

namespace ReinforcementLearningDrive {

class AckermannSteeringActor : public ROS2Actor {
 public:
  using Ptr = std::shared_ptr<AckermannSteeringActor>;
  AckermannSteeringActor(const rclcpp::Node::SharedPtr& node, std::string actor_name);
  AckermannSteeringActor(const rclcpp::Node::SharedPtr& node, std::string actor_name, double frequency);

 private:
  void m_initialize();
  void m_reset() override;
  void m_predictPose(const Command& twist, double dt) override;

  double m_steering_angle;
  double m_wheel_base;
};

}  // namespace ReinforcementLearningDrive

#endif  // __ACKERMANN_STEERING_ACTOR_H__

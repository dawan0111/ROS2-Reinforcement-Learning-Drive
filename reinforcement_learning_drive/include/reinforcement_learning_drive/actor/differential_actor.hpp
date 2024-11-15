#ifndef __DIFFERENTIAL_ACTOR_H__
#define __DIFFERENTIAL_ACTOR_H__

#include "reinforcement_learning_drive/actor/ros2_actor.hpp"

namespace ReinforcementLearningDrive {

class DifferentialActor : public ROS2Actor {
 public:
  using Ptr = std::shared_ptr<DifferentialActor>;
  DifferentialActor(const rclcpp::Node::SharedPtr& node, std::string actor_name);
  DifferentialActor(const rclcpp::Node::SharedPtr& node, std::string actor_name, double frequency);

 private:
  void m_initialize();
  void m_reset() override;
  void m_predictPose(const Command& twist, double dt) override;

  double m_wheel_base;
};

}  // namespace ReinforcementLearningDrive

#endif  // __DIFFERENTIAL_ACTOR_H__

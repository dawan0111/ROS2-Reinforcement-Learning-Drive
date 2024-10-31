#ifndef __ROS2_ENVIRONMENT_H__
#define __ROS2_ENVIRONMENT_H__

#include <rclcpp/rclcpp.hpp>
#include "reinforcement_learning_drive/environment/environment.hpp"

namespace ReinforcementLearningDrive {
class ROS2Environment : public Environment {
  using Ptr = std::shared_ptr<ROS2Environment>;

 public:
  ROS2Environment(const rclcpp::Node::SharedPtr& node) : Environment(), m_node(node){};
  virtual EnvStatus getStatus(const std::shared_ptr<Actor>& actor) const = 0;

 protected:
  rclcpp::Node::SharedPtr m_node;

 private:
  virtual void initEnvironment() = 0;
  virtual bool collisionCheck(const std::shared_ptr<Actor>& actor) const = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __ROS2_ENVIRONMENT_H__
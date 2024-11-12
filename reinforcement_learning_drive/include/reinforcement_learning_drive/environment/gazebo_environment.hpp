#ifndef __GAZEBO_ENVIRONMENT_H__
#define __GAZEBO_ENVIRONMENT_H__

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "reinforcement_learning_drive/environment/ros2_environment.hpp"

namespace ReinforcementLearningDrive {
class GazeboEnvironment : public ROS2Environment {
 public:
  GazeboEnvironment(const rclcpp::Node::SharedPtr&);
  EnvStatus getStatus(const std::shared_ptr<Actor>& actor) const override;

 private:
  void initEnvironment() override;
  bool collisionCheck(const std::shared_ptr<Actor>& actor) const override;

  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr m_spawn_client_;
};
}  // namespace ReinforcementLearningDrive

#endif  // __GAZEBO_ENVIRONMENT_H__
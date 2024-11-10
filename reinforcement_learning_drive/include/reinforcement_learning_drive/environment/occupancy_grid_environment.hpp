#ifndef __OCCUPANCY_GRID_ENVIRONMENT_H__
#define __OCCUPANCY_GRID_ENVIRONMENT_H__

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "reinforcement_learning_drive/environment/ros2_environment.hpp"

namespace ReinforcementLearningDrive {
class OccupancyGridEnvironment : public ROS2Environment {
 public:
  OccupancyGridEnvironment(const rclcpp::Node::SharedPtr&);
  EnvStatus getStatus(const std::shared_ptr<Actor>& actor) const override;

 private:
  void initEnvironment() override;
  bool collisionCheck(const std::shared_ptr<Actor>& actor) const override;

  double m_max_distance;
  std::vector<double> m_ray_angles;

  std::vector<std::pair<double, double>> getScanData(const std::shared_ptr<Actor>& actor) const;
  std::pair<int16_t, int16_t> getCellXY(const geometry_msgs::msg::Pose& pose) const;
  std::pair<double, double> getRayCast(int16_t start_x, int16_t start_y, double start_yaw, double max_distance,
                                       double ray_angle) const;
  std::string m_map_service_name;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_subscribe;
  nav_msgs::msg::OccupancyGrid m_map;
};
}  // namespace ReinforcementLearningDrive

#endif  // __OCCUPANCY_GRID_ENVIRONMENT_H__
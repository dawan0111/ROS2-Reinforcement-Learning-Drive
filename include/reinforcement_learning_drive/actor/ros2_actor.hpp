#ifndef __ROS2_ACTOR_H__
#define __ROS2_ACTOR_H__

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
class ROS2Actor : public Actor {

 public:
  using Ptr = std::shared_ptr<ROS2Actor>;
  ROS2Actor(const rclcpp::Node::SharedPtr&);

  void run(const Command& twist) override;

 protected:
  rclcpp::Node::SharedPtr m_node;
  void m_visualize() override;
  void m_reset() override;

 private:
  void m_collisionVisualize();
  void m_tfPublish();
  void m_markerVisualize();

  std::string m_pose_topic_name;
  std::string m_control_topic_name;
  std::string m_pose_service_name;
  std::string m_tf_name;

  bool m_enable_topic{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_pose_sub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr m_polygon_pub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

}  // namespace ReinforcementLearningDrive

#endif  // __ROS2_ACTOR_H__
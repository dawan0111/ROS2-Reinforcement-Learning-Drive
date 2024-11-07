#include "reinforcement_learning_drive/actor/ros2_actor.hpp"

namespace ReinforcementLearningDrive {
ROS2Actor::ROS2Actor(const rclcpp::Node::SharedPtr& node) : Actor() {
  m_pose_topic_name = "/bicycle_steering_controller/odometry";
  m_control_topic_name = "/cmd_vel";
  m_pose_service_name = "/bicycle_steering_controller/update";
  m_tf_name = "base_link";

  m_node = node;
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);

  m_marker_pub = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
  m_polygon_pub = m_node->create_publisher<geometry_msgs::msg::PolygonStamped>("collision", 10);

  if (m_enable_topic) {
    m_control_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    m_pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        m_pose_topic_name, rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          auto& position = msg->pose.pose.position;
          auto& orientation = msg->pose.pose.orientation;
          m_updatePose(std::move(msg->pose));
        });
  }
};
void ROS2Actor::m_reset() {
  Actor::m_reset();
}
void ROS2Actor::run(const Command& twist) {
  if (m_enable_topic) {
    m_control_pub->publish(twist);
  }
  Actor::run(twist);

  // RCLCPP_INFO(m_node->get_logger(), "Score: %f", m_actor_status->score);
  // RCLCPP_INFO(m_node->get_logger(), "Distance: %f", m_actor_status->goal_distance);
  // RCLCPP_INFO(m_node->get_logger(), "Angle: %f", m_actor_status->goal_angle);
}

void ROS2Actor::m_visualize() {
  m_tfPublish();
  m_collisionVisualize();
  m_markerVisualize();
}

void ROS2Actor::m_tfPublish() {
  geometry_msgs::msg::TransformStamped transform_stamped;

  const auto& pose = getCurrentPose();
  // RCLCPP_INFO(m_node->get_logger(), "x: %f", pose->pose.position.x);
  // RCLCPP_INFO(m_node->get_logger(), "y: %f", pose->pose.position.y);
  // RCLCPP_INFO(m_node->get_logger(), "z: %f", pose->pose.position.z);

  transform_stamped.header.stamp = m_node->get_clock()->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = m_tf_name;
  transform_stamped.transform.translation.x = pose->pose.position.x;
  transform_stamped.transform.translation.y = pose->pose.position.y;
  transform_stamped.transform.translation.z = pose->pose.position.z + 0.05;

  transform_stamped.transform.rotation = pose->pose.orientation;

  m_tf_broadcaster->sendTransform(transform_stamped);
}

void ROS2Actor::m_markerVisualize() {
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker score_marker;
  score_marker.header.frame_id = m_tf_name;
  score_marker.header.stamp = m_node->get_clock()->now();
  score_marker.ns = "score";
  score_marker.id = 0;
  score_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  score_marker.action = visualization_msgs::msg::Marker::ADD;
  score_marker.pose.position.x = 0.0;
  score_marker.pose.position.y = 0.0;
  score_marker.pose.position.z = 1.0;
  score_marker.scale.z = 0.25;
  score_marker.color.a = 1.0;
  score_marker.color.r = 1.0;
  score_marker.color.g = 1.0;
  score_marker.color.b = 1.0;
  score_marker.text = "Score: " + std::to_string(m_actor_status->score);

  visualization_msgs::msg::Marker scan_marker;
  scan_marker.header.frame_id = m_tf_name;
  scan_marker.header.stamp = m_node->now();
  scan_marker.ns = "scan_data";
  scan_marker.id = 1;
  scan_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  scan_marker.action = visualization_msgs::msg::Marker::ADD;
  scan_marker.scale.x = 0.05;
  scan_marker.color.a = 1.0;

  for (const auto& [distance, angle] : m_actor_status->scan_data) {
    geometry_msgs::msg::Point start_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 0.0;

    geometry_msgs::msg::Point end_point;
    double actual_distance = std::isinf(distance) ? 10.0 : distance * 10.0;
    end_point.x = actual_distance * std::cos(angle);
    end_point.y = actual_distance * std::sin(angle);
    end_point.z = 0.0;

    if (std::isinf(distance)) {
      scan_marker.color.r = 0.0;
      scan_marker.color.g = 0.0;
      scan_marker.color.b = 1.0;

    } else {
      scan_marker.color.r = 1.0;
      scan_marker.color.g = 0.0;
      scan_marker.color.b = 0.0;
    }

    scan_marker.points.push_back(start_point);
    scan_marker.points.push_back(end_point);

    marker_array.markers.push_back(scan_marker);
    scan_marker.points.clear();
    ++scan_marker.id;
  }
  marker_array.markers.push_back(score_marker);

  m_marker_pub->publish(marker_array);
}

void ROS2Actor::m_collisionVisualize() {
  const auto collision_space = getCollisionArea();

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.stamp = m_node->get_clock()->now();
  polygon_msg.header.frame_id = m_tf_name;

  for (size_t i = 0; i < collision_space.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = collision_space[i];
    p.y = collision_space[i + 1];
    p.z = 0.0;
    polygon_msg.polygon.points.push_back(p);
  }

  m_polygon_pub->publish(polygon_msg);
}

}  // namespace ReinforcementLearningDrive
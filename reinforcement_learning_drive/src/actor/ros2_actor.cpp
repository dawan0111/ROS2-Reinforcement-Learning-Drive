#include "reinforcement_learning_drive/actor/ros2_actor.hpp"

namespace ReinforcementLearningDrive {
ROS2Actor::ROS2Actor(const rclcpp::Node::SharedPtr& node, std::string actor_name) : Actor(actor_name), m_node(node) {
  initialize();
};

ROS2Actor::ROS2Actor(const rclcpp::Node::SharedPtr& node, std::string actor_name, double control_frequency)
    : Actor(actor_name, control_frequency), m_node(node) {
  initialize();
};

void ROS2Actor::initialize() {

  m_pose_topic_name = m_name + "/odom";
  m_control_topic_name = m_name + "/cmd_vel";
  m_pose_service_name = m_name + "/update";
  m_scan_topic_name = m_name + "/scan";
  m_tf_name = m_name + "_virtual";

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);
  m_actor_cb_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions pub_options;

  sub_options.callback_group = m_actor_cb_group;
  pub_options.callback_group = m_actor_cb_group;

  m_marker_pub = m_node->create_publisher<visualization_msgs::msg::MarkerArray>(m_name + "/visual", 1, pub_options);
  m_polygon_pub = m_node->create_publisher<geometry_msgs::msg::PolygonStamped>(m_name + "/collision", 1, pub_options);

  if (m_enable_topic) {
    m_control_pub =
        m_node->create_publisher<geometry_msgs::msg::Twist>(m_control_topic_name, rclcpp::QoS(1), pub_options);
    m_pose_sub = m_node->create_subscription<nav_msgs::msg::Odometry>(
        m_pose_topic_name, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          auto& position = msg->pose.pose.position;
          auto& orientation = msg->pose.pose.orientation;
          updatePose(std::move(msg->pose));
        },
        sub_options);
    m_scan_sub = m_node->create_subscription<sensor_msgs::msg::LaserScan>(
        m_scan_topic_name, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::unique_lock<std::mutex> lock(m_status_mtx, std::defer_lock);
          std::vector<std::pair<double, double>> scan_data;
          double current_angle = 0.0;
          m_max_scan_distance = msg->range_max;

          for (const auto& range : msg->ranges) {
            double normalized_angle = std::atan2(std::sin(current_angle), std::cos(current_angle));
            scan_data.emplace_back(std::isfinite(range) ? range / m_max_scan_distance : 1.0, normalized_angle);
            current_angle += msg->angle_increment;
          }
          lock.lock();
          m_actor_status->scan_data = scan_data;
        },
        sub_options);
  }
}

void ROS2Actor::m_reset() {
  if (m_enable_topic) {
    RCLCPP_INFO(m_node->get_logger(), "RESET!!");
    Command twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    m_control_pub->publish(twist);
  }
  Actor::m_reset();
}
void ROS2Actor::run(const Command& twist) {
  if (m_enable_topic) {
    m_control_pub->publish(twist);
  }
  Actor::run(twist);
}

void ROS2Actor::m_visualize() {
  m_tfPublish();
  m_collisionVisualize();
  m_markerVisualize();
}

void ROS2Actor::m_tfPublish() {
  geometry_msgs::msg::TransformStamped transform_stamped;

  const auto& pose = getCurrentPose();

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
  std::unique_lock<std::mutex> lock(m_status_mtx);
  auto score = m_actor_status->score;
  lock.unlock();

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
  score_marker.text = m_tf_name + " Score: " + std::to_string(score);

  visualization_msgs::msg::Marker scan_marker;
  scan_marker.header.frame_id = m_tf_name;
  scan_marker.header.stamp = m_node->now();
  scan_marker.ns = "scan_data";
  scan_marker.id = 1;
  scan_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  scan_marker.action = visualization_msgs::msg::Marker::ADD;
  scan_marker.scale.x = 0.05;
  scan_marker.color.a = 1.0;

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
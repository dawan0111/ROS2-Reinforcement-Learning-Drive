#include "reinforcement_learning_drive/environment/gazebo_environment.hpp"

namespace ReinforcementLearningDrive {
GazeboEnvironment::GazeboEnvironment(const rclcpp::Node::SharedPtr& node) : ROS2Environment(node) {
  m_spawn_client_ = m_node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  initEnvironment();
}

void GazeboEnvironment::initEnvironment() {
  if (!m_spawn_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(m_node->get_logger(), "Service not available, waiting...");
    return;
  }

  std::string stage_model;
  uint16_t num_actors;
  uint16_t stage_row;
  double stage_x;
  double stage_y;

  m_node->declare_parameter<std::string>("stage_model", "/home/kdw/building_editor_models/racing2/model.sdf");
  m_node->declare_parameter<double>("stage_x", 15.0);
  m_node->declare_parameter<double>("stage_y", 15.0);
  m_node->declare_parameter<uint16_t>("stage_row", 4);

  m_node->get_parameter("stage_model", stage_model);
  m_node->get_parameter("num_actors", num_actors);
  m_node->get_parameter("stage_x", stage_x);
  m_node->get_parameter("stage_y", stage_y);
  m_node->get_parameter("stage_row", stage_row);

  std::ifstream sdf_file(stage_model);
  if (!sdf_file) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to open SDF file.");
    exit(0);
  }

  std::stringstream sdf_stream;
  sdf_stream << sdf_file.rdbuf();
  sdf_file.close();

  for (int i = 0; i < num_actors; ++i) {
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = "enviornment" + std::to_string(i + 1);
    request->xml = sdf_stream.str();
    request->robot_namespace = "/";
    request->initial_pose.position.x = stage_x * (i % stage_row);
    request->initial_pose.position.y = stage_y * static_cast<int>(i / stage_row);
    request->initial_pose.position.z = 0.0;

    RCLCPP_INFO(m_node->get_logger(), "call enviornment.");

    auto future = m_spawn_client_->async_send_request(request);

    try {
      if (rclcpp::spin_until_future_complete(m_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(m_node->get_logger(), "Success spawn enviornment.");
      } else {
        RCLCPP_INFO(m_node->get_logger(), "Failed spawn enviornment.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(m_node->get_logger(), "Service call failed.");
    }
  }

  m_is_initialized = true;
}

EnvStatus GazeboEnvironment::getStatus(const std::shared_ptr<Actor>& actor) const {
  EnvStatus env_status;
  return env_status;
}

bool GazeboEnvironment::collisionCheck(const std::shared_ptr<Actor>& actor) const {
  return false;
}

}  // namespace ReinforcementLearningDrive
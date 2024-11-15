#include "reinforcement_learning_drive/environment/gazebo_environment.hpp"
#include "reinforcement_learning_drive/actor/actor.hpp"
#include "reinforcement_learning_drive/reward/reward.hpp"

namespace ReinforcementLearningDrive {
GazeboEnvironment::GazeboEnvironment(const rclcpp::Node::SharedPtr& node) : ROS2Environment(node) {
  rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  m_spawn_client_ = m_node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  m_reset_client_ = m_node->create_client<gazebo_msgs::srv::SetEntityState>("/actor/set_entity_state", qos_profile);
}

void GazeboEnvironment::initParameter() {
  m_node->declare_parameter<std::string>("stage_model", "/home/kdw/building_editor_models/racing2/model.sdf");
  m_node->declare_parameter<std::string>("actor_model", "/home/kdw/building_editor_models/racing2/actor.sdf");

  m_node->declare_parameter<double>("stage_x", 10.0);
  m_node->declare_parameter<double>("stage_y", 6.0);
  m_node->declare_parameter<double>("initial_x", 0.0);
  m_node->declare_parameter<double>("initial_y", 0.75);
  m_node->declare_parameter<uint16_t>("stage_row", 4);

  m_node->get_parameter("stage_model", m_stage_model);
  m_node->get_parameter("actor_model", m_actor_model);
  m_node->get_parameter("num_actors", m_num_actors);
  m_node->get_parameter("stage_x", m_stage_x);
  m_node->get_parameter("stage_y", m_stage_y);
  m_node->get_parameter("initial_x", m_initial_x);
  m_node->get_parameter("initial_y", m_initial_y);
  m_node->get_parameter("stage_row", m_stage_row);
}

void GazeboEnvironment::initEnvironment() {
  initParameter();
  initGazeboSpawn();

  m_is_initialized = true;
}

void GazeboEnvironment::initGazeboSpawn() {
  if (!m_spawn_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(m_node->get_logger(), "Service not available, waiting...");
    return;
  }

  std::ifstream model_file(m_stage_model);
  std::ifstream actor_file(m_actor_model);

  std::stringstream model_sdf_stream;
  model_sdf_stream << model_file.rdbuf();
  model_file.close();

  std::stringstream actor_sdf_stream;
  actor_sdf_stream << actor_file.rdbuf();
  actor_file.close();

  for (int i = 0; i < m_num_actors; ++i) {
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = "enviornment" + std::to_string(i + 1);
    request->xml = model_sdf_stream.str();
    request->robot_namespace = "/";
    request->initial_pose.position.x = m_stage_x * (i % m_stage_row);
    request->initial_pose.position.y = m_stage_y * static_cast<int>(i / m_stage_row);
    request->initial_pose.position.z = 0.0;

    auto future = m_spawn_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(m_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(m_node->get_logger(), "Failed spawn enviornment.");
    }

    request->xml = actor_sdf_stream.str();
    request->name = m_actor_vec[i]->getName();
    request->robot_namespace = m_actor_vec[i]->getName();

    request->initial_pose.position.x = m_stage_x * (i % m_stage_row) + m_initial_x;
    request->initial_pose.position.y = m_stage_y * static_cast<int>(i / m_stage_row) + m_initial_y;
    request->initial_pose.position.z = 0.05;

    auto actor_future = m_spawn_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(m_node, actor_future) == rclcpp::FutureReturnCode::SUCCESS) {
      geometry_msgs::msg::PoseWithCovariance actor_inital_pose;
      actor_inital_pose.pose = request->initial_pose;
      m_actor_vec[i]->updateInitialPose(std::move(actor_inital_pose));
    } else {
      RCLCPP_INFO(m_node->get_logger(), "Failed spawn actor.");
    }
  }
}

EnvStatus GazeboEnvironment::getStatus(const std::shared_ptr<Actor>& actor) const {
  EnvStatus env_status;
  const auto actor_status = actor->getActorStatus();

  env_status.collision = collisionCheck(actor, actor_status) && !actor->reset;
  env_status.scan_data = actor_status->scan_data;

  return env_status;
}

bool GazeboEnvironment::collisionCheck(const std::shared_ptr<Actor>& actor,
                                       const std::shared_ptr<EnvStatus>& status) const {
  const double threshold_distance = 0.25;

  for (const auto& [distance, angle] : status->scan_data) {
    if (distance <= threshold_distance && distance > 0.1) {
      return true;
    }
  }

  return false;
}

bool GazeboEnvironment::resetActor(const std::shared_ptr<Actor>& actor) {
  if (actor->reset) {
    return false;
  }
  RCLCPP_INFO(m_node->get_logger(), "Reset Call: %s", actor->getName().c_str());

  actor->reset = true;

  auto initial_pose = actor->getInitialPose();
  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

  request->state.name = actor->getName();
  request->state.pose = initial_pose.pose;
  request->state.reference_frame = "world";

  auto future = m_reset_client_->async_send_request(
      request, [this, actor](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture response) {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        RCLCPP_INFO(m_node->get_logger(), "Reset Finish: %s", actor->getName().c_str());
        actor->reset = false;
      });

  return true;
}

}  // namespace ReinforcementLearningDrive
#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

namespace ReinforcementLearningDrive {
ReinforcementLearningDrive::ReinforcementLearningDrive(const rclcpp::NodeOptions& options)
    : Node("reinforcement_learning_drive", options) {}

void ReinforcementLearningDrive::initialize() {
  this->declare_parameter<std::string>("actor_prefix", "actor");
  this->declare_parameter<uint16_t>("num_actors", 1);
  this->declare_parameter<double>("control_frequency", 30);

  this->get_parameter("actor_prefix", m_actor_prefix);
  this->get_parameter("num_actors", m_num_actors);
  this->get_parameter("control_frequency", m_control_frequency);

  RCLCPP_INFO(this->get_logger(), "========== Parameter ==========");
  RCLCPP_INFO(this->get_logger(), "num_actors: %d", m_num_actors);
  RCLCPP_INFO(this->get_logger(), "actor_prefix: %s", m_actor_prefix.c_str());
  RCLCPP_INFO(this->get_logger(), "control_frequency: %f", m_control_frequency);

  m_environment = std::make_shared<GazeboEnvironment>(shared_from_this());
  m_reward = std::make_shared<ScanReward>(shared_from_this());

  for (int i = 0; i < m_num_actors; i++) {
    auto actor = std::make_shared<AckermannSteeringActor>(shared_from_this(), m_actor_prefix + std::to_string(i + 1),
                                                          m_control_frequency);
    actor->setEnvironment(m_environment);
    actor->run(m_current_twist);

    m_environment->addActor(std::move(actor));
  }

  /**
   * Configure Environment
   */
  m_environment->initEnvironment();
  m_environment->setReward(m_reward);

  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = nullptr;

  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { m_current_twist = *msg; });

  m_timer = this->create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() -> void {
        auto actors = m_environment->getActorVec();
        std::for_each(std::execution::par, actors.begin(), actors.end(), [this](std::shared_ptr<Actor>& actor) {
          actor->run(m_current_twist);
          actor->debug();
        });
      },
      client_cb_group_);

  m_drive_service_server = this->create_service<Drive>(
      "drive_service",
      std::bind(&ReinforcementLearningDrive::executeService, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, client_cb_group_);
  m_multi_drive_service_server = this->create_service<MultiDrive>(
      "multi_drive_service",
      std::bind(&ReinforcementLearningDrive::executeMultiService, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, client_cb_group_);
}

void ReinforcementLearningDrive::executeService(const std::shared_ptr<Drive::Request> request,
                                                std::shared_ptr<Drive::Response> response) {
  auto start_time = std::chrono::steady_clock::now();
  auto actors = m_environment->getActorVec();
  m_current_twist = request->target_velocity;
  actors[0]->run(m_current_twist);

  response->state = getActorStatus(actors[0]);

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  RCLCPP_INFO(this->get_logger(), "Service executed in %f ms", duration / 1000.0);
}

void ReinforcementLearningDrive::executeMultiService(const std::shared_ptr<MultiDrive::Request> request,
                                                     std::shared_ptr<MultiDrive::Response> response) {
  auto start_time = std::chrono::steady_clock::now();
  std::atomic<int16_t> index{0};
  tbb::concurrent_vector<State> vec(m_num_actors);
  auto actors = m_environment->getActorVec();

  std::vector<size_t> indices(m_num_actors);
  std::iota(indices.begin(), indices.end(), 0);

  std::for_each(std::execution::par, indices.begin(), indices.end(), [this, &request, &vec, &actors](size_t i) {
    auto& actor = actors[i];
    auto twist = request->target_velocity[i];
    actor->run(twist);
    vec[i] = getActorStatus(actor);
  });

  std::vector<State> states(vec.begin(), vec.end());
  response->state = states;

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  RCLCPP_INFO(this->get_logger(), "Service executed in %f ms", duration / 1000.0);
}

ReinforcementLearningDrive::State ReinforcementLearningDrive::getActorStatus(std::shared_ptr<Actor>& actor) {
  State state;
  const auto& actor_status = actor->getActorStatus();

  std::vector<double> flat_scan_data;
  flat_scan_data.reserve(actor_status->scan_data.size());
  for (const auto& [distance, angle] : actor_status->scan_data) {
    flat_scan_data.push_back(std::isinf(distance) ? 1.0 : distance);
  }

  state.name = actor->getName();
  state.scan_data = flat_scan_data;
  state.pose = actor_status->pose.pose;
  state.score = actor_status->score;
  state.done = actor_status->collision;
  state.goal_distance = actor_status->goal_distance;
  state.goal_angle = actor_status->goal_angle;

  return state;
}
}  // namespace ReinforcementLearningDrive
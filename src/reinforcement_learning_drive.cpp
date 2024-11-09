#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

namespace ReinforcementLearningDrive {
ReinforcementLearningDrive::ReinforcementLearningDrive(const rclcpp::NodeOptions& options)
    : Node("reinforcement_learning_drive", options) {}

void ReinforcementLearningDrive::initialize() {
  auto actor = std::make_shared<AckermannSteeringActor>(shared_from_this(), "actor1");
  auto actor2 = std::make_shared<AckermannSteeringActor>(shared_from_this(), "actor2");
  m_environment = std::make_shared<OccupancyGridEnvironment>(shared_from_this());
  m_reward = std::make_shared<ScanReward>(shared_from_this());

  m_actors.push_back(std::move(actor));
  m_actors.push_back(std::move(actor2));

  /**
   * Configure Environment
   */
  m_environment->setReward(m_reward);

  /**
   * Configure Actor
   */
  for (auto& actor : m_actors) {
    actor->setEnvironment(m_environment);
    actor->run(m_current_twist);
  }

  // m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
  //     "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { m_current_twist = *msg; });
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = nullptr;

  m_timer = this->create_wall_timer(
      std::chrono::milliseconds(33),
      [this]() -> void {
        std::for_each(std::execution::par, m_actors.begin(), m_actors.end(),
                      [](std::shared_ptr<Actor>& actor) { actor->debug(); });
      },
      timer_cb_group_);

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

  m_current_twist = request->target_velocity;
  m_actors[0]->run(m_current_twist);

  response->state = getActorStatus(m_actors[0]);

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // 실행 시간 로그 출력
  RCLCPP_INFO(this->get_logger(), "Service executed in %f ms", duration / 1000.0);
}

void ReinforcementLearningDrive::executeMultiService(const std::shared_ptr<MultiDrive::Request> request,
                                                     std::shared_ptr<MultiDrive::Response> response) {
  auto start_time = std::chrono::steady_clock::now();
  std::atomic<int16_t> index{0};
  tbb::concurrent_vector<State> vec;

  std::for_each(std::execution::par, m_actors.begin(), m_actors.end(),
                [this, &request, &vec, &index](std::shared_ptr<Actor>& actor) {
                  auto twist = request->target_velocity[index.fetch_add(1)];
                  actor->run(twist);
                  vec.push_back(getActorStatus(actor));
                });

  std::vector<State> states(vec.begin(), vec.end());
  response->state = states;

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // 실행 시간 로그 출력
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
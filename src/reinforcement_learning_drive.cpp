#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

namespace ReinforcementLearningDrive {
ReinforcementLearningDrive::ReinforcementLearningDrive(const rclcpp::NodeOptions& options)
    : Node("reinforcement_learning_drive", options) {}

void ReinforcementLearningDrive::initialize() {
  m_actor = std::make_shared<AckermannSteeringActor>(shared_from_this());
  m_environment = std::make_shared<OccupancyGridEnvironment>(shared_from_this());
  m_reward = std::make_shared<ScanReward>(shared_from_this());

  /**
   * Configure Environment
   */
  m_environment->setReward(m_reward);

  /**
   * Configure Actor
   */
  m_actor->setEnvironment(m_environment);
  m_actor->run(m_current_twist);

  // m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
  //     "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { m_current_twist = *msg; });
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = nullptr;

  m_timer = this->create_wall_timer(
      std::chrono::milliseconds(33), [this]() -> void { m_actor->debug(); }, timer_cb_group_);

  m_drive_service_server = this->create_service<Drive>(
      "drive_service",
      std::bind(&ReinforcementLearningDrive::executeService, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, client_cb_group_);
}

void ReinforcementLearningDrive::executeService(const std::shared_ptr<Drive::Request> request,
                                                std::shared_ptr<Drive::Response> response) {
  // 서비스 실행 시작 시간 기록
  auto start_time = std::chrono::steady_clock::now();

  // 서비스 요청을 통해 받은 속도 명령으로 처리
  m_current_twist = request->target_velocity;
  m_actor->run(m_current_twist);
  const auto& actor_status = m_actor->getActorStatus();

  // 스캔 데이터를 평면화하여 응답에 저장
  std::vector<double> flat_scan_data;
  flat_scan_data.reserve(actor_status->scan_data.size());
  for (const auto& [distance, angle] : actor_status->scan_data) {
    flat_scan_data.push_back(std::isinf(distance) ? 1.0 : distance);
  }

  // 응답 데이터 설정
  response->state.scan_data = flat_scan_data;
  response->state.pose = actor_status->pose.pose;
  response->state.score = actor_status->score;
  response->state.done = actor_status->collision;
  response->state.goal_distance = actor_status->goal_distance;
  response->state.goal_angle = actor_status->goal_angle;

  // 서비스 실행 종료 시간 기록
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  // 실행 시간 로그 출력
  RCLCPP_INFO(this->get_logger(), "Service executed in %f ms", duration / 1000.0);
}
}  // namespace ReinforcementLearningDrive
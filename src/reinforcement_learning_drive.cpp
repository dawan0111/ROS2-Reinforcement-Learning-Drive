#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

namespace ReinforcementLearningDrive {
ReinforcementLearningDrive::ReinforcementLearningDrive(const rclcpp::NodeOptions& options)
    : Node("reinforcement_learning_drive", options) {}

void ReinforcementLearningDrive::initialize() {
  m_actor = std::make_shared<AckermannSteeringActor>(shared_from_this());
  m_environment = std::make_shared<OccupancyGridEnvironment>(shared_from_this());
  m_reward = std::make_shared<PathTrackingReward>(shared_from_this(), true);

  /**
   * Configure Environment
   */
  m_environment->setReward(m_reward);

  /**
   * Configure Actor
   */
  m_actor->setEnvironment(m_environment);

  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { m_current_twist = *msg; });

  m_timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() -> void { m_actor->run(m_current_twist); });
  m_drive_action_server = rclcpp_action::create_server<Drive>(
      this, "drive_action",
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Drive::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandleDrive>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandleDrive> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&ReinforcementLearningDrive::executeAction, this, _1), goal_handle}.detach();
      });
}

void ReinforcementLearningDrive::executeAction(const std::shared_ptr<GoalHandleDrive> goal_handle) {
  const auto goal = goal_handle->get_goal();

  m_current_twist = goal->target_velocity;
  m_actor->run(m_current_twist);
  const auto& actor_status = m_actor->getActorStatus();
  std::vector<double> flat_scan_data;

  flat_scan_data.reserve(actor_status->scan_data.size());

  for (const auto& [distance, angle] : actor_status->scan_data) {
    if (std::isinf(distance)) {
      flat_scan_data.push_back(1.0);
    } else {
      flat_scan_data.push_back(distance);
    }
  }

  auto result = std::make_shared<Drive::Result>();
  result->scan_data = flat_scan_data;  // 예시 데이터
  result->pose = actor_status->pose.pose;
  result->score = actor_status->score;
  result->done = actor_status->collision;
  result->goal_distance = actor_status->goal_distance;
  result->goal_angle = actor_status->goal_angle;

  goal_handle->succeed(result);
}

}  // namespace ReinforcementLearningDrive
#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

namespace ReinforcementLearningDrive {
ReinforcementLearningDrive::ReinforcementLearningDrive(const rclcpp::NodeOptions& options)
    : Node("reinforcement_learning_drive", options) {}

void ReinforcementLearningDrive::initialize() {
  m_actor = std::make_shared<AckermannSteeringActor>(shared_from_this());
  m_environment = std::make_shared<OccupancyGridEnvironment>(shared_from_this());

  m_actor->setEnvironment(m_environment);

  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { m_current_twist = *msg; });

  m_timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() -> void { m_actor->run(m_current_twist); });
}

}  // namespace ReinforcementLearningDrive
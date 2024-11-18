#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
Actor::Actor(std::string actor_name, double control_frequency) {
  m_pose = std::make_shared<Pose>();
  m_actor_status = std::make_shared<EnvStatus>();
  m_name = actor_name;
  m_dt = 1.0 / control_frequency;
}

void Actor::run(const Command& twist) {
  m_predictPose(twist, m_dt);
  std::this_thread::sleep_for(std::chrono::duration<double>(m_dt));

  auto actor_status = m_env->getStatus(shared_from_this());
  *m_actor_status = std::move(actor_status);

  if (m_actor_status->collision) {
    m_reset();
  }
};

void Actor::debug() {
  m_visualize();
}

void Actor::m_reset() {
  m_env->resetActor(shared_from_this());
}

double Actor::quatToYaw() const {
  double siny_cosp = 2.0 * (m_pose->pose.orientation.w * m_pose->pose.orientation.z +
                            m_pose->pose.orientation.x * m_pose->pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (m_pose->pose.orientation.y * m_pose->pose.orientation.y +
                                  m_pose->pose.orientation.z * m_pose->pose.orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion Actor::yawToQuat(double yaw) const {
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::cos(yaw * 0.5);
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);

  return orientation;
}
}  // namespace ReinforcementLearningDrive
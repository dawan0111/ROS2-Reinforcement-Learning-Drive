#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
void Actor::run(const Command& twist) {
  m_predictPose(twist, m_dt);
  *m_actor_status = m_env->getStatus(shared_from_this());
  m_visualize();
  if (m_actor_status->collision) {
    m_reset();
  }
};

void Actor::m_reset() {
  Pose pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  m_updatePose(std::move(pose));
}
}  // namespace ReinforcementLearningDrive
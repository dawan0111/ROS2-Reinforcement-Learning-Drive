#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
void Actor::run(const Command& twist) {
  m_predictPose(twist, m_dt);
  *m_actor_status = m_env->getStatus(shared_from_this());
  m_visualize();
};
}  // namespace ReinforcementLearningDrive
#ifndef __REWARD_H__
#define __REWARD_H__

#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
class Actor;
class Reward {
 public:
  Reward() = default;
  virtual bool calculateReward(const std::shared_ptr<Actor>& actor, const std::shared_ptr<EnvStatus>& status) = 0;
  double getScore() { return m_score; };

  /**
   * TODO: Custom Data로 변경
   */
  double getDistanceToGoal() { return m_goal_distance; };
  double getDistanceToAngle() { return m_goal_angle; };

  virtual void reset() = 0;

 protected:
  bool m_is_initialized{false};
  double m_goal_distance{0.0};
  double m_goal_angle{0.0};
  double m_score{0.0};
};
}  // namespace ReinforcementLearningDrive

#endif  // __REWARD_H__
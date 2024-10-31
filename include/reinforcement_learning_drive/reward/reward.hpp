#ifndef __REWARD_H__
#define __REWARD_H__

namespace ReinforcementLearningDrive {
class Reward {
 public:
  Reward() = default;
  virtual double calculateReward() = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __REWARD_H__
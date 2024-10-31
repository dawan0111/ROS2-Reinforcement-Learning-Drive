#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "reinforcement_learning_drive/actor/actor.hpp"

namespace ReinforcementLearningDrive {
class Actor;
struct EnvStatus {
  int32_t score;
  std::vector<std::pair<double, double>> scan_data;
  bool collision;
  EnvStatus() : score(0), scan_data({}), collision(false){};
  EnvStatus(int32_t score_, std::vector<std::pair<double, double>> scan_data_)
      : score(score_), scan_data(scan_data_), collision(false){};
};
class Environment {
 public:
  Environment(){};
  virtual EnvStatus getStatus(const std::shared_ptr<Actor>& actor) const = 0;

 protected:
  bool m_is_initialized{false};

 private:
  virtual void initEnvironment() = 0;
  virtual bool collisionCheck(const std::shared_ptr<Actor>& actor) const = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __ENVIRONMENT_H__
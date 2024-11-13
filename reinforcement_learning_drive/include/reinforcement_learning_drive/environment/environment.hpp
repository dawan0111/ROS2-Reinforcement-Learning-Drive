#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__
#include "geometry_msgs/msg/pose_with_covariance.hpp"
namespace ReinforcementLearningDrive {
class Actor;
class Reward;
struct EnvStatus {
  double score;
  std::vector<std::pair<double, double>> scan_data;
  geometry_msgs::msg::PoseWithCovariance pose;
  bool collision;
  double goal_distance;
  double goal_angle;

  EnvStatus() : score(0), scan_data({}), collision(false){};
  EnvStatus(int32_t score_, std::vector<std::pair<double, double>> scan_data_)
      : score(score_), scan_data(scan_data_), collision(false){};
};
class Environment {
 public:
  Environment(){};
  virtual EnvStatus getStatus(const std::shared_ptr<Actor>& actor) const = 0;

  void setReward(std::shared_ptr<Reward> reward) { m_reward = reward; }
  std::shared_ptr<Reward> getReward() const { return m_reward; }

 protected:
  bool m_is_initialized{false};
  std::shared_ptr<Reward> m_reward{nullptr};

 private:
  virtual void initEnvironment() = 0;
  virtual bool collisionCheck(const std::shared_ptr<Actor>& actor, const std::shared_ptr<EnvStatus>& status) const = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __ENVIRONMENT_H__
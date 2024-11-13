#ifndef __ACTOR_H__
#define __ACTOR_H__
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "reinforcement_learning_drive/environment/environment.hpp"

namespace ReinforcementLearningDrive {
class Environment;
struct EnvStatus;
class Actor : public std::enable_shared_from_this<Actor> {
 public:
  using Pose = geometry_msgs::msg::PoseWithCovariance;
  using Command = geometry_msgs::msg::Twist;

  Actor(std::string actor_name, double control_frequency = 30.0);

  std::shared_ptr<Pose> getCurrentPose() const { return m_pose; };
  const std::shared_ptr<EnvStatus> getActorStatus() {
    std::lock_guard<std::mutex> lock(m_status_mtx);
    return std::make_shared<EnvStatus>(*m_actor_status);
  };

  void setEnvironment(std::shared_ptr<Environment> env) { m_env = env; }
  std::shared_ptr<Environment> getEnvironment() const { return m_env; }

  std::vector<double> getCollisionArea() { return m_collision_space; };
  std::string getName() { return m_name; }

  void debug();
  virtual void run(const Command& twist);

  double quatToYaw() const;

  geometry_msgs::msg::Quaternion yawToQuat(double yaw) const;

 protected:
  virtual void m_reset();
  virtual void m_visualize() = 0;
  void m_updatePose(Pose&& pose) { m_pose->pose = pose.pose; };

  std::shared_ptr<Environment> m_env;
  std::shared_ptr<EnvStatus> m_actor_status;
  std::vector<double> m_collision_space;

  std::string m_name;
  double m_dt{0.033};

  std::mutex m_status_mtx;

 private:
  std::shared_ptr<Pose> m_pose;
  virtual void m_predictPose(const Command& twist, double dt) = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __ACTOR_H__
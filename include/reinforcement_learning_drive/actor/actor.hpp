#ifndef __ACTOR_H__
#define __ACTOR_H__
#include <cmath>
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

  Actor() {
    m_pose = std::make_shared<Pose>();
    m_actor_status = std::make_shared<EnvStatus>();
  };

  std::shared_ptr<Pose> getCurrentPose() const { return m_pose; };
  std::shared_ptr<Environment> getEnvironment() const { return m_env; }
  void setEnvironment(std::shared_ptr<Environment> env) { m_env = env; }

  double quatToYaw() const {
    double siny_cosp = 2.0 * (m_pose->pose.orientation.w * m_pose->pose.orientation.z +
                              m_pose->pose.orientation.x * m_pose->pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (m_pose->pose.orientation.y * m_pose->pose.orientation.y +
                                    m_pose->pose.orientation.z * m_pose->pose.orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Quaternion yawToQuat(double yaw) const {
    geometry_msgs::msg::Quaternion orientation;
    orientation.w = std::cos(yaw * 0.5);
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);

    return orientation;
  }

  virtual void run(const Command& twist);
  std::vector<double> getCollisionArea() { return m_collision_space; };

 protected:
  virtual void m_reset();
  virtual void m_visualize() = 0;
  void m_updatePose(Pose&& pose) { m_pose->pose = pose.pose; };

  std::shared_ptr<Environment> m_env;
  std::shared_ptr<EnvStatus> m_actor_status;
  std::vector<double> m_collision_space;
  double m_dt{0.01};

 private:
  std::shared_ptr<Pose> m_pose;
  virtual void m_predictPose(const Command& twist, double dt) = 0;
};
}  // namespace ReinforcementLearningDrive

#endif  // __ACTOR_H__
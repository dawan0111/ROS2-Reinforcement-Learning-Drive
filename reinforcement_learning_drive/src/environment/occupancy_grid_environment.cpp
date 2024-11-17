#include "reinforcement_learning_drive/environment/occupancy_grid_environment.hpp"
#include "reinforcement_learning_drive/actor/actor.hpp"
#include "reinforcement_learning_drive/reward/reward.hpp"

namespace ReinforcementLearningDrive {
OccupancyGridEnvironment::OccupancyGridEnvironment(const rclcpp::Node::SharedPtr& node) : ROS2Environment(node) {
  m_map_service_name = "/map";
  double angle_step = M_PI / 18.0;
  for (double angle = -M_PI / 2; angle <= M_PI / 2; angle += angle_step) {
    m_ray_angles.push_back(angle);
  }
};
EnvStatus OccupancyGridEnvironment::getStatus(const std::shared_ptr<Actor>& actor) const {
  EnvStatus status{0, {}};

  if (m_map.data.size() == 0) {
    RCLCPP_WARN(m_node->get_logger(), "map data is empty!");
    return status;
  }
  auto actor_status = actor->getActorStatus();

  bool collision = collisionCheck(actor, actor_status);

  if (collision) {
    m_reward->reset();
  } else {
    m_reward->calculateReward(actor, actor_status);
  }

  status.score = m_reward->getScore();
  status.goal_angle = m_reward->getDistanceToAngle();
  status.goal_distance = m_reward->getDistanceToGoal();
  status.scan_data = getScanData(actor);
  status.collision = collision;
  status.pose = *actor->getCurrentPose();

  return status;
}

std::vector<std::pair<double, double>> OccupancyGridEnvironment::getScanData(
    const std::shared_ptr<Actor>& actor) const {
  std::vector<std::pair<double, double>> scan_data;
  auto [x, y] = getCellXY(actor->getCurrentPose()->pose);
  double yaw = actor->quatToYaw();
  double max_scan_distance = actor->getMaxScanDistance();

  for (const auto& ray_angle : m_ray_angles) {
    scan_data.push_back(getRayCast(x, y, yaw, max_scan_distance, ray_angle));
  }

  return scan_data;
}

void OccupancyGridEnvironment::initEnvironment() {
  auto qos = rclcpp::QoS(10).transient_local();
  m_map_subscribe = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      m_map_service_name, qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) -> void {
        m_map.header = map->header;
        m_map.data = map->data;
        m_map.info = map->info;
        m_is_initialized = true;
      });
}

bool OccupancyGridEnvironment::collisionCheck(const std::shared_ptr<Actor>& actor,
                                              const std::shared_ptr<EnvStatus>& status) const {
  const auto& pose = actor->getCurrentPose()->pose;
  const auto& collision_space = actor->getCollisionArea();

  auto [x, y] = getCellXY(pose);
  double yaw = actor->quatToYaw();

  auto collision_pose = pose;
  double start_x = pose.position.x;
  double start_y = pose.position.y;

  for (size_t i = 0; i < collision_space.size(); i += 2) {
    double local_x = collision_space[i];
    double local_y = collision_space[i + 1];

    collision_pose.position.x += (local_x * std::cos(yaw) - local_y * std::sin(yaw));
    collision_pose.position.y += (local_x * std::sin(yaw) + local_y * std::cos(yaw));

    auto [map_x, map_y] = getCellXY(collision_pose);

    if (map_x >= 0 && map_x < m_map.info.width && map_y >= 0 && map_y < m_map.info.height) {
      int index = map_y * m_map.info.width + map_x;

      if (m_map.data[index] > 50) {
        return true;
      }
    }

    collision_pose.position.x = start_x;
    collision_pose.position.y = start_y;
  }
  return false;
}

bool OccupancyGridEnvironment::resetActor(const std::shared_ptr<Actor>& actor) {
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;

  actor->updatePose(std::move(pose));
  return true;
}

std::pair<int16_t, int16_t> OccupancyGridEnvironment::getCellXY(const geometry_msgs::msg::Pose& pose) const {
  double resolution = m_map.info.resolution;
  double origin_x = m_map.info.origin.position.x;
  double origin_y = m_map.info.origin.position.y;

  int16_t x = static_cast<int>((pose.position.x - origin_x) / resolution);
  int16_t y = static_cast<int>((pose.position.y - origin_y) / resolution);

  return {x, y};
}
std::pair<double, double> OccupancyGridEnvironment::getRayCast(int16_t start_x, int16_t start_y, double start_yaw,
                                                               double max_distance, double ray_angle) const {
  int16_t width = m_map.info.width;
  int16_t height = m_map.info.height;
  double resolution = m_map.info.resolution;

  double cast_angle = start_yaw + ray_angle;
  double direction_x = cos(cast_angle);
  double direction_y = sin(cast_angle);

  for (double distance = 0.0; distance <= max_distance; distance += resolution) {
    int16_t x = start_x + static_cast<int>(distance * direction_x / resolution);
    int16_t y = start_y + static_cast<int>(distance * direction_y / resolution);

    if (x < 0 || x >= width || y < 0 || y >= height) {
      break;
    }

    int index = y * width + x;

    if (m_map.data[index] > 50) {
      return {distance / max_distance, ray_angle};
    }
  }

  return {std::numeric_limits<double>::infinity(), ray_angle};
}

};  // namespace ReinforcementLearningDrive
#include "reinforcement_learning_drive/reinforcement_learning_drive.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReinforcementLearningDrive::ReinforcementLearningDrive>(rclcpp::NodeOptions());
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
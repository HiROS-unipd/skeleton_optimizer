#include "skeleton_optimizer/Optimizer.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::skeletons::Optimizer>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

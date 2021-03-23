// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_optimizer/Optimizer.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_skeleton_optimizer";
  ros::init(argc, argv, node_name);

  hiros::optimizer::Optimizer optimizer;
  optimizer.start();

  ros::spin();

  return 0;
}

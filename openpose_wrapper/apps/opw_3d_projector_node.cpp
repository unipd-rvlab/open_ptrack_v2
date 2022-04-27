// ROS
#include <ros/ros.h>

// Internal dependencies
#include "openpose_wrapper/projector.h"

int main(int argc, char* argv[])
{
  std::string node_name = "open_ptrack_opw3d";
  ros::init(argc, argv, node_name);

  open_ptrack::opw3d::Projector projector;
  projector.configure();
  projector.start();

  exit(EXIT_SUCCESS);
}

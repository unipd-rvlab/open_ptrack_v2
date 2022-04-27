// ROS
#include <ros/ros.h>

// Internal dependencies
#include "openpose_wrapper/wrapper.h"

int main(int argc, char* argv[])
{
  std::string node_name = "open_ptrack_opw";
  ros::init(argc, argv, node_name);

  open_ptrack::opw::Wrapper wrapper;
  wrapper.configure();
  wrapper.start();

  exit(EXIT_SUCCESS);
}

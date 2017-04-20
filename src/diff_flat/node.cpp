#include <ros/ros.h>
#include "diff_flat/controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_flat_node");
  ros::NodeHandle nh;

  diff_flat::Controller Thing;

  ros::spin();

  return 0;
}

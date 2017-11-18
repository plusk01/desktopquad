#include <ros/ros.h>
#include "mcl/mcl.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "mcl");

  // instantiate an object
  mcl::MCL mcl;

  // Tell MCL estimator that a tick has passed
  ros::Rate r(100);
  while(ros::ok())
  {
    mcl.tick();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
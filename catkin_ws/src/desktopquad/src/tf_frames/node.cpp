#include <ros/ros.h>
#include "tf_frames/TFFrames.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "tf_frames");

  // instantiate an object
  tf_frames::TFFrames frames;

  // Tell TFFrames that a tick has passed
  ros::Rate r(100);
  while(ros::ok())
  {
    frames.tick();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
#include <ros/ros.h>
#include "tf_frames/tf_frames.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "tf_frames");

  // instantiate an object
  tf_frames::TFFrames frames;

  ros::spin();
  return 0;
}
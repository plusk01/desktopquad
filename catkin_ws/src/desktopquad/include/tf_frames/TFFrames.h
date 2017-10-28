#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace tf_frames
{

  class TFFrames
  {
  public:
    TFFrames();
    void tick();

  private:
    // Node handles, publishers, subscribers
    ros::NodeHandle nh_;

    // ROS params
    double platform_height_ = 0;

    // ROS tf listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_br_;
  };

}
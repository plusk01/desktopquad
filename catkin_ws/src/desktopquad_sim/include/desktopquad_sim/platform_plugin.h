#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#define PI 3.141592

namespace gazebo
{
  class PlatformPlugin : public ModelPlugin {
  public:
    PlatformPlugin();
    ~PlatformPlugin();

  protected:

    void Reset() override;
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void OnUpdate(const common::UpdateInfo & _info);

  private:
    // ROS Stuff
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br;

    // Pointer to the gazebo items.
    physics::LinkPtr base_link_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection_;

  };
} // namespace gazebo
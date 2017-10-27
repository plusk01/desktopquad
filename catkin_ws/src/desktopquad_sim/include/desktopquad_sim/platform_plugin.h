#pragma once

#include <string>
#include <iostream>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>

namespace gazebo
{
  class PlatformPlugin : public ModelPlugin {
  public:
    PlatformPlugin();
    ~PlatformPlugin();

  protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void OnUpdate(const common::UpdateInfo & _info);

  private:
    // ROS Stuff
    tf::TransformBroadcaster br_;

    // Pointer to the gazebo items.
    physics::LinkPtr base_link_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection_;

    double base_height_ = 0;
  };
} // namespace gazebo
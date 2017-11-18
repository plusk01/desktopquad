#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseArray.h>

namespace mcl
{

  // coordiante/point struct
  typedef struct { double x, y, z; } point_t;

  // ========================

  // Particle filter parameter struct
  typedef struct
  {
    // number of particles
    int M;

  } parameters_t;

  // ========================

  // Particle struct
  typedef struct
  {

  } particle_t;

  // ========================

  class MCL
  {
  public:
    MCL();

    void tick();

  private:
    // Node handles
    ros::NodeHandle nh_;

    // Publishers and Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber is_flying_sub_;
    ros::Publisher map_pub_;

    // ROS tf listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_br_;

    // Paramters
    bool is_flying_;
    parameters_t params_;

    // Map parameters
    std::string map_frame_;
    std::unordered_map<int, point_t> landmarks_;

    // methods
    void init();
    void sample_motion_model();
    void perceptual_model();

    void create_map(XmlRpc::XmlRpcValue& map);

    void publish_map();
    void publish_particles();
  };

}
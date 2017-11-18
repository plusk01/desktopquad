#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseArray.h>

namespace mcl
{
  const unsigned int NSTATE = 6;

  // coordinate/point struct
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

    // position
    double x, y, z;

    // orientation (Euler)
    double R, P, Y;

  } state_t;

  // ========================

  // Particle struct
  typedef struct
  {
    state_t state;

    // likelihood associated with this
    // particle for the resampling step.
    double w;

  } particle_t;

  // ========================

  class MCL
  {
  public:
    MCL();

    // Tick -- this method is the heartbeat of the localization filter.
    // Every tick the propagation step is performed. If new measurements
    // are available, they will be used to resample the particles. This 
    // allows the particles with the closest estimate of reality to live.
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

    // Particle filter data members
    std::vector<particle_t> particles_;
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

    Eigen::VectorXd VectorXd_rand(int length, double low, double high);
  };

}
#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <random>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Imu.h>
#include "aruco_localization/MarkerMeasurementArray.h"
#include "aruco_localization/MarkerMeasurement.h"

#include "mcl/particle.h"
#include "mcl/motionmodels/motion_model.h"
#include "mcl/motionmodels/sdncv.h"
#include "mcl/motionmodels/mech.h"

namespace mcl
{
  // coordinate/point struct
  typedef struct { double x, y, z; } point_t;

  // ========================

  // Particle filter parameter struct
  typedef struct
  {
    // number of particles
    int M;

    // initial volume
    std::vector<double> valid_x, valid_y, valid_z;
    std::vector<double> valid_R, valid_P, valid_Y;

    // initial volume
    std::vector<double> initial_x, initial_y, initial_z;
    std::vector<double> initial_R, initial_P, initial_Y;

  } parameters_t;

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
    ros::Publisher map_pub_;
    ros::Publisher particles_pub_;
    ros::Subscriber meas_sub_;
    ros::Subscriber is_flying_sub_;
    ros::Subscriber imu_sub_;

    // ROS tf listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_br_;

    // Measurement queues
    std::queue<aruco_localization::MarkerMeasurementArrayConstPtr> landmark_measurements_;

    // Particle filter data members
    std::vector<ParticlePtr> particles_;
    parameters_t params_;

    // Map parameters
    std::string working_frame_; // this is the frame that MCL is performed in (aruco)
    std::unordered_map<int, point_t> landmarks_;

    // motion model
    MotionModelPtr mm_;

    // methods
    double perceptual_model(const aruco_localization::MarkerMeasurement& z, ParticlePtr& p);

    void create_map(XmlRpc::XmlRpcValue& xMap);
    void init_particles();

    void measurements_cb(const aruco_localization::MarkerMeasurementArrayConstPtr& msg);
    void imu_cb(const sensor_msgs::ImuConstPtr&  msg);

    void publish_map();
    void publish_particles();

    double mvnpdf(const Eigen::VectorXd& x, const Eigen::VectorXd& mu, const Eigen::MatrixXd& Sigma);
  };

}

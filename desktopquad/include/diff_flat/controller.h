#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <rosflight_msgs/Command.h>

#include <desktopquad/ControllerConfig.h>

namespace diff_flat {

typedef struct {
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;

  double ax;
  double ay;
  double az;

  double throttle;
} state_t;

typedef struct {
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double u;
  double v;
  double w;
} max_t;

typedef struct {
  int n; // state vector size
  int p; // input/control vector size
  int q; // output vector size

  // A \in R^{n x n}
  // B \in R^{n x p}
  // C \in R^{q x n}
  // D \in R^{q x p}
} dims_t;

typedef struct {
  Eigen::MatrixXd F;
  Eigen::MatrixXd N;
  Eigen::MatrixXd K;
} lqr_gains_t;

class Controller {

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber cmd_sub_;

  ros::Publisher command_pub_;

  // Paramters
  double thrust_eq_;
  double mass_;
  double max_thrust_;
  double drag_constant_;
  bool is_flying_;

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<desktopquad::ControllerConfig> _server;
  dynamic_reconfigure::Server<desktopquad::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(desktopquad::ControllerConfig &config, uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  max_t max_ = {};
  rosflight_msgs::Command command_;
  state_t xc_ = {}; // command
  double prev_time_;
  uint8_t control_mode_;
  dims_t dims_ = {}; // dimensions of the system
  lqr_gains_t lqr_gains_ = {};

  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
  double sgn(double x);
  Eigen::Matrix3d rot_psi(double psi);
};

} // namespace diff_flat
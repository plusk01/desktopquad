#include <diff_flat/controller.h>
#include <stdio.h>

namespace diff_flat {

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // retrieve global MAV params (mass and max thrust)
  ros::NodeHandle nh_mav(ros::this_node::getNamespace());
  mass_ = nh_mav.param<double>("mass", 3.81);
  max_thrust_ = nh_mav.param<double>("max_F", 74.0);
  drag_constant_ = nh_mav.param<double>("linear_mu", 0.1);
  thrust_eq_= (9.80665 * mass_) / max_thrust_;
  is_flying_ = false;

  max_.roll = nh_private_.param<double>("max_roll", 0.15);
  max_.pitch = nh_private_.param<double>("max_pitch", 0.15);
  max_.yaw_rate = nh_private_.param<double>("max_yaw_rate", 45.0*M_PI/180.0);
  max_.throttle = nh_private_.param<double>("max_throttle", 1.0);
  max_.u = nh_private_.param<double>("max_u", 1.0);
  max_.v = nh_private_.param<double>("max_v", 1.0);
  max_.w = nh_private_.param<double>("max_w", 1.0);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  cmd_sub_ = nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);

  command_pub_ = nh_.advertise<fcu_common::Command>("command", 1);

  dims_.n = 7;
  dims_.p = 4;
  dims_.q = 4;
}


void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg) {
  static double prev_time = 0;
  if(prev_time == 0) {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);
  xhat_.theta = xhat_.theta;
  xhat_.psi = xhat_.psi;

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(is_flying_) {
    computeControl(dt);
    publishCommand();
  } else {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg) {
  is_flying_ = msg->data;
}


void Controller::cmdCallback(const fcu_common::CommandConstPtr &msg) {
  switch(msg->mode)
 {
    case fcu_common::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      xc_.pn = msg->x;
      xc_.pe = msg->y;
      xc_.pd = msg->F;
      xc_.psi = msg->z;
      control_mode_ = msg->mode;
      break;
    case fcu_common::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      xc_.u = msg->x;
      xc_.v = msg->y;
      xc_.pd = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    case fcu_common::Command::MODE_XACC_YACC_YAWRATE_AZ:
      xc_.ax = msg->x;
      xc_.ay = msg->y;
      xc_.az = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("ros_copter/controller: Unhandled command message of type %d", msg->mode);
      break;
  }
}


void Controller::reconfigure_callback(ros_copter::ControllerConfig &config, uint32_t level) {

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;
  max_.u = config.max_u;
  max_.v = config.max_v;
  max_.w = config.max_w;

  ROS_INFO("new gains");

  resetIntegrators();
}


void Controller::computeControl(double dt) {
  if(dt <= 0.0000001) {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == fcu_common::Command::MODE_XPOS_YPOS_YAW_ALTITUDE) {

    // --------------------------------
    // Manage setpoint to create proper
    // feedforward and reference state
    // --------------------------------

    // build the reference vector: r = [pn pe pd psi]
    Eigen::VectorXd r;
    r << xc_.pn, xc_.pe, xc_.pd, xc_.psi;

    Eigen::VectorXd x_r = Eigen::VectorXd::Zero(dims_.n, 1);
    Eigen::VectorXd u_r = Eigen::VectorXd::Zero(dims_.p, 1);

    // Find the equilibrium state: xr = [pn pe pd pndot pedot pddot psi]
    x_r = lqr_gains_.F*r;

    // Find the equilibrium input: ur = [pnddot peddot pdddot psidot]
    Eigen::VectorXd gvec; gvec << 0, 0, 0, 9.81, 0;
    u_r = lqr_gains_.N*r - gvec;

    // --------------------------------
    // Linear-Quadratic Regulator (LQR)
    // --------------------------------

    // Create state vector used by LQR
    Eigen::VectorXd x_lqr;
    x_lqr << xhat_.pn, xhat_.pe, xhat_.pd, xhat_.u, xhat_.v, xhat_.w, xhat_.psi;
    // Note: u, v, w should be rotated into the inertial frame. But since this
    // is just a setpoint controller (going to positions with ending vel = 0)
    // it actually doesn't matter because we want LQR to regulate them to 0.

    // Form error-state to regulate to zero: x_tilde = x - x_r
    Eigen::VectorXd x_tilde = x_lqr - x_r;

    // LQR Feedback Control Law
    Eigen::VectorXd u_tilde = -lqr_gains_.K*x_tilde;

    // Add feedforward input term
    Eigen::VectorXd u = u_tilde + u_r;

    // --------------------------------
    // Nonlinear mapping from u to nu
    // --------------------------------

    // Our LQR controller gives outputs as u = [pnddot peddot pdddot psidot]

  }


  if(mode_flag == fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE) {
    // Pack up and send the command
    command_.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }
}

void Controller::publishCommand() {
  command_pub_.publish(command_);
}

void Controller::resetIntegrators() {

}

double Controller::saturate(double x, double max, double min) {
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

double Controller::sgn(double x) {
  return (x >= 0.0) ? 1.0 : -1.0;
}

} // namespace diff_flat


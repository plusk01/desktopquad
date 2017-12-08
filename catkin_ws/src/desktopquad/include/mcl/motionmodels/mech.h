#pragma once
/**
 * Mechanized Motion Model: Integrates IMU

Based on the motion model used in:
    Beard: Quadrotor Notes, P. 22
*/

#include "mcl/motionmodels/motion_model.h"

#include "lib/eigenmvn.h"

namespace mcl {

  class MECH : public MotionModel
  {
  public:
    MECH(double std_x, double std_y, double std_z, double std_ax, double std_ay, double std_az)
    {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();

      // Build zero-mean WSS linear velocity process
      Eigen::Vector3d vel_var(std::pow(std_x,2), std::pow(std_y,2), std::pow(std_z,2));
      eta_vel_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, vel_var.asDiagonal(), true);

      // Build zero-mean WSS angular velocity process
      Eigen::Vector3d avel_var(std::pow(std_ax,2), std::pow(std_ay,2), std::pow(std_az,2));
      eta_avel_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, avel_var.asDiagonal(), true);
    }

    void sample(ParticlePtr& p, double dt) override
    {
      // Gate the accelerometer measurements so the particle doesn't go crazy.
      // This is particularly important at the beginning of Gazebo simulations when
      // the IMU gives back either zeros or very noisy samples.
      if (acc_.norm() < (1.0 - acc_margin_)*gvec_.norm() || acc_.norm() > (1.0 + acc_margin_)*gvec_.norm()) return;

      // Sample the random variables to create velocity inputs
      Eigen::Vector3d V     = eta_vel_->samples(1);
      Eigen::Vector3d Omega = eta_avel_->samples(1);

      // Create rotation matrix from camera to working frame
      // Note: Eigen uses active rotations, so this line should technically read:
      //    p->quat.inverse().toRotationMatrix().transpose()
      Eigen::Matrix3d R_c2w = p->quat.toRotationMatrix();
      
      // angular velocity (camera frame)
      p->omega = gyro_ - gyro_b_ + Omega;

      // velocity (camera frame)
      Eigen::Vector3d b_vel;
      b_vel << p->omega(2)*p->vel(1) - p->omega(1)*p->vel(2);
               p->omega(0)*p->vel(2) - p->omega(2)*p->vel(0);
               p->omega(1)*p->vel(0) - p->omega(0)*p->vel(1);

      // acceleration due to the force of gravity in the body frame
      Eigen::Vector3d ag = R_c2w.transpose() * gvec_;

      p->vel += (acc_ - acc_b_ + b_vel + ag)*dt + V;

      // position (working frame)
      p->pos += R_c2w * p->vel*dt;

      // orientation (working to camera frame)
      Eigen::Quaterniond qdot =  p->quat * Eigen::Quaterniond(0, p->omega(0)*dt, p->omega(1)*dt, p->omega(2)*dt);
      p->quat.w() += 0.5*qdot.w();
      p->quat.x() += 0.5*qdot.x();
      p->quat.y() += 0.5*qdot.y();
      p->quat.z() += 0.5*qdot.z();
      p->quat.normalize();
    }

    void set_imu(Eigen::Vector3d acc, Eigen::Vector3d gyro)
    {
      acc_ = acc;
      gyro_ = gyro;
    }

    void set_biases(Eigen::Vector3d bacc, Eigen::Vector3d bgyro)
    {
      acc_b_ = bacc;
      gyro_b_ = bgyro;
    }

  private:
    // input stochastic processes
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_vel_; // linear velocity
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_avel_; // angular velocity

    // acceleration due to gravity vector (defined in inertial NED frame)
    const Eigen::Vector3d gvec_ = Eigen::Vector3d(0, 0, 9.81);

    // Allowable margin (percentage) on accelerometer values +/- expected (gvec_)
    double acc_margin_ = 0.20;

    // Updated by the caller
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_b_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_b_ = Eigen::Vector3d::Zero();
  };

}


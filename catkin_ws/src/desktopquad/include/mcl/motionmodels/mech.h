#pragma once
/**
 * Mechanized Motion Model: Integrates IMU & Gyro

Based on the motion model used in:
    'Beard: Quadrotor Notes, P. 22
*/

#include "mcl/motionmodels/motion_model.h"

#include "lib/eigenmvn.h"

namespace mcl {

  class MECH : public MotionModel
  {
  public:
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_b = Eigen::Vector3d::Zero();
    const double g = 9.81;
    MECH(double std_x, double std_y, double std_z, double std_ax, double std_ay, double std_az)
    {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();

      // Build zero-mean WSS linear acceleration process
      Eigen::Vector3d vel_var(std::pow(std_x,2), std::pow(std_y,2), std::pow(std_z,2));
      eta_vel_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, vel_var.asDiagonal(), true);

      // Build zero-mean WSS angular acceleration process
      Eigen::Vector3d avel_var(std::pow(std_ax,2), std::pow(std_ay,2), std::pow(std_az,2));
      eta_avel_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, avel_var.asDiagonal(), true);
    }

    void sample(ParticlePtr& p, double dt) override
    {

      if (acc(2) > -20 && acc(2) < -6) {
          // Sample the random variables to create velocity inputs
          Eigen::Vector3d V     = eta_vel_->samples(1);
          Eigen::Vector3d Omega = eta_avel_->samples(1);

          // Cosine and Sine of Heading Angles
          auto euler = p->quat.toRotationMatrix().transpose().eulerAngles(0,1,2);
          double cphi = std::cos(euler(0));
          double sphi = std::sin(euler(0));
          double cth = std::cos(euler(1));
          double sth = std::sin(euler(1));
          double cpsi = std::cos(euler(2));
          double spsi = std::sin(euler(2));
            
          // angular velocity (measurement frame)
          p->omega = gyro - gyro_b + Omega;  

          // velocity (measurement frame)
          Eigen::Vector3d b_vel;
          b_vel << p->omega(2)*p->vel(1) - p->omega(1)*p->vel(2) - g*sth,
                   p->omega(0)*p->vel(2) - p->omega(2)*p->vel(0) + g*cth*sphi,
                   p->omega(1)*p->vel(0) - p->omega(0)*p->vel(1) + g*cth*cphi;

          p->vel = (acc-acc_b+b_vel)*dt + V;

          // position (working frame)
          // Note: Eigen uses active rotations, so this line should technically read:
          //    p->quat.inverse().toRotationMatrix().transpose() * p->vel*dt
          p->pos += p->quat.toRotationMatrix() * p->vel*dt;

          // orientation (working to measurement frame)
          Eigen::Quaterniond qdot =  p->quat * Eigen::Quaterniond(0, p->omega(0)*dt, p->omega(1)*dt, p->omega(2)*dt);
          p->quat.w() += 0.5*qdot.w();
          p->quat.x() += 0.5*qdot.x();
          p->quat.y() += 0.5*qdot.y();
          p->quat.z() += 0.5*qdot.z();
          p->quat.normalize();
      }
    }

  private:
    // input stochastic processes
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_vel_; // linear acceleration
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_avel_; // angular acceleration
      
  };

}


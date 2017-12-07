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
    const Eigen::Vector3d gvec = Eigen::Vector3d(0, 0, 9.80556);
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
      // if (acc(2) < -20 || acc(2) > -6) return;
      if (acc.norm() < 0.8*gvec.norm() || acc.norm() > 1.2*gvec.norm()) return;

      // Sample the random variables to create velocity inputs
      // Eigen::Vector3d V     = Eigen::Vector3d::Zero();
      // Eigen::Vector3d Omega = Eigen::Vector3d::Zero();

      Eigen::Vector3d V     = eta_vel_->samples(1);
      Eigen::Vector3d Omega = eta_avel_->samples(1);

      // Create rotation matrix from measurement to working frame
      // Note: Eigen uses active rotations, so this line should technically read:
      //    p->quat.inverse().toRotationMatrix().transpose()
      Eigen::Matrix3d R_m2w = p->quat.toRotationMatrix();
      
      // angular velocity (measurement frame)
      // p->omega = Eigen::Vector3d::Zero();
      p->omega = gyro - gyro_b + Omega;

      // velocity (measurement frame)
      Eigen::Vector3d b_vel;
      b_vel << p->omega(2)*p->vel(1) - p->omega(1)*p->vel(2);
               p->omega(0)*p->vel(2) - p->omega(2)*p->vel(0);
               p->omega(1)*p->vel(0) - p->omega(0)*p->vel(1);

      // acceleration due to the force of gravity in the body frame
      Eigen::Vector3d ag = R_m2w.transpose() * gvec;

      p->vel += (acc - acc_b + b_vel + ag)*dt + V;
      // std::cout << std::left << std::setw(8) << "ag: "     << ag.transpose() << "\t\t";
      // std::cout << std::left << std::setw(8) << "b_vel: "  << b_vel.transpose() << "\n";
      // std::cout << std::left << std::setw(8) << "acc: "    << acc.transpose() << "\n";
      // std::cout << std::left << std::setw(8) << "accel: "  << (acc-acc_b+b_vel+ag).transpose() << "\t\t";
      // std::cout << std::left << std::setw(8) << "p->vel: " << p->vel.transpose();
      // std::cout << std::endl;

      // position (working frame)
      p->pos += R_m2w * p->vel*dt;

      // orientation (working to measurement frame)
      Eigen::Quaterniond qdot =  p->quat * Eigen::Quaterniond(0, p->omega(0)*dt, p->omega(1)*dt, p->omega(2)*dt);
      p->quat.w() += 0.5*qdot.w();
      p->quat.x() += 0.5*qdot.x();
      p->quat.y() += 0.5*qdot.y();
      p->quat.z() += 0.5*qdot.z();
      p->quat.normalize();
    }

  private:
    // input stochastic processes
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_vel_; // linear velocity
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> eta_avel_; // angular velocity
      
  };

}


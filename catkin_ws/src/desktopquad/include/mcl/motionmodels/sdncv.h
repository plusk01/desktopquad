#pragma once
/**
Stochastically-Driven Nearly-Constant Velocity motion model

Based on the motion model used in 'MonoSLAM', Davison et al., 2007.
*/

#include "mcl/motionmodels/motion_model.h"

#include "lib/eigenmvn.h"

namespace mcl {

  class SDNCV : public MotionModel
  {
  public:
    SDNCV(double std_x, double std_y, double std_z, double std_ax, double std_ay, double std_az)
    {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();

      // Build zero-mean WSS linear acceleration process
      Eigen::Vector3d accel_var(std::pow(std_x,2), std::pow(std_y,2), std::pow(std_z,2));
      accel_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, accel_var.asDiagonal(), true);

      // Build zero-mean WSS angular acceleration process
      Eigen::Vector3d alpha_var(std::pow(std_ax,2), std::pow(std_ay,2), std::pow(std_az,2));
      alpha_ = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, alpha_var.asDiagonal(), true);
    }

    void sample(ParticlePtr& p, double dt) override
    {
      Eigen::Vector3d V = accel_->samples(1)*dt;

      // position propagation
      p->pos += p->quat.toRotationMatrix() * (p->vel + V)*dt;

      // velocity propagation
      p->vel += V;
    }

  private:
    // input stochastic processes
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> accel_; // linear acceleration
    std::shared_ptr<Eigen::EigenMultivariateNormal<double>> alpha_; // angular acceleration
      
  };

}
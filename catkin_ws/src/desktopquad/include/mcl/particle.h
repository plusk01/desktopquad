#pragma once

#include <memory>

#include <Eigen/Dense>

namespace mcl {

  class Particle
  {
  public:
    Particle(double x, double y, double z, double R, double P, double Y)
    {
      pos = Eigen::Vector3d(x, y, z);

      // Build quaternion from RPY
      // Tait-Bryan (3-2-1;Z-Y-X) sequence common in aerospace
      // confirmed with `.eulerAngles(2,1,0)`
      quat = Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX());

      vel = Eigen::Vector3d(0, 0, 0);
      omega = Eigen::Vector3d(0, 0, 0);

      w = 0;
    }

    // position (in working frame)
    Eigen::Vector3d pos;
    // orientation (from working to measurement frame)
    Eigen::Quaterniond quat;
    // velocity (in measurement frame)
    Eigen::Vector3d vel;
    // angular rates (in measurement frame)
    Eigen::Vector3d omega;

    // likelihood associated with this
    // particle for the resampling step.
    double w;

    // 'Structures Having Eigen Memebers' -- Alignment issue when using `new`
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  typedef std::shared_ptr<Particle> ParticlePtr;

}
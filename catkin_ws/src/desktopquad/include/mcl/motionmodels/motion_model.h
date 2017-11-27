#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "mcl/particle.h"

namespace mcl {

  enum MotionModels {
    MM_SDNCV,
    MM_MECH
  };

  class MotionModel
  {
  public:
    virtual void sample(ParticlePtr& p, double dt) = 0;
  };

  typedef std::shared_ptr<MotionModel> MotionModelPtr;
}

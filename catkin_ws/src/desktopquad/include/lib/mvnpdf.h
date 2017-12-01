#pragma once

#include <stdexcept>

#include <Eigen/Dense>

namespace rv {

  class mvnpdf
  {
  public:
    mvnpdf(Eigen::MatrixXd Sigma) {
      // error checking
      if (Sigma.rows() != Sigma.cols()) {
        throw std::invalid_argument("Sigma must be square!");
      }

      // store covariance and information matrices
      Sigma_ = Sigma;
      SigmaInv_ = Sigma.inverse();

      // dimension of the probability space
      int k = Sigma_.rows();

      // Calculate normalization constant for Gaussian pdf
      eta_ = 1.0/std::sqrt( std::pow(2*M_PI,k) * Sigma_.determinant() );

      // Calculate constants for log-likelihood function
      nu1_ = -0.5*std::log( Sigma_.determinant() );
      nu2_ = -0.5*k*std::log(2*M_PI);
    }

    double logprob(const Eigen::VectorXd& x, const Eigen::VectorXd& mu)
    {
      // error checking
      if ((x.size() != mu.size()) || (x.size() != Sigma_.rows())) {
        throw std::invalid_argument("x and mu must be the same size!");
      }

      // Calculate the Mahalanobis distance (squared)
      Eigen::VectorXd residual = x - mu;
      double mahal_sq = residual.transpose() * SigmaInv_ * residual;

      // Log-likelihood function (natural log of a mvn pdf)
      return nu1_ + -0.5*mahal_sq + nu2_;
    }

    double probability(const Eigen::VectorXd& x, const Eigen::VectorXd& mu)
    {
      // error checking
      if ((x.size() != mu.size()) || (x.size() != Sigma_.rows())) {
        throw std::invalid_argument("x and mu must be the same size!");
      }

      // Kernel of a gaussian
      Eigen::VectorXd residual = x - mu;
      double gaussian = std::exp( -0.5 * residual.transpose() * SigmaInv_ * residual );

      return eta_*gaussian;
    }

  private:
    // normalization constant for Gaussian pdf
    double eta_;

    // constants in log-likelihood Gaussian pdf function
    double nu1_;
    double nu2_;

    // Covariance and Information matrices
    Eigen::MatrixXd Sigma_;
    Eigen::MatrixXd SigmaInv_;
    
  };

}
#ifndef __ALPHA_FILTER_HPP__
#define __ALPHA_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Core>

namespace EstimateTools {

class AlphaFilter{
  public:
    AlphaFilter(Eigen::VectorXd &alpha);
    AlphaFilter(double alpha);

    ~AlphaFilter(){
    }

    void processSample(const Eigen::VectorXd& x, Eigen::VectorXd &x_filtered);

  private:
    double single_alpha_;
    Eigen::VectorXd alpha_;
    Eigen::VectorXd one_minus_alpha_;

    bool use_single_alpha_;
    bool init_;
    bool verbose_;

    Eigen::VectorXd x_filtered_prev_;
};

}

#endif

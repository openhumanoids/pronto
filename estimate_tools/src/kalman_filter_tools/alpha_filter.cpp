#include <estimate_tools/alpha_filter.hpp>
using namespace Eigen;
using namespace std;

namespace EstimateTools {

AlphaFilter::AlphaFilter(Eigen::VectorXd &alpha): 
     alpha_(alpha) {
  init_ = false;
  one_minus_alpha_ = Eigen::VectorXd::Constant(alpha.size(), 1) - alpha;
  verbose_ = false;
  use_single_alpha_ = false;
}

AlphaFilter::AlphaFilter(double alpha): 
     single_alpha_(alpha) {
  init_ = false;
  verbose_ = false;
  use_single_alpha_ = true;
}

void AlphaFilter::processSample(const Eigen::VectorXd& x, 
                    Eigen::VectorXd &x_filtered){
  if (!init_) {
     x_filtered = x;
     x_filtered_prev_ = x_filtered;
     init_ = true;
     return;
  }

  if (!use_single_alpha_) {
    if (x.size() != alpha_.size()) {
      throw std::runtime_error("alpha filter dimension mismatch.");
    }
    x_filtered <<  alpha_.array() * x_filtered_prev_.array()  + one_minus_alpha_.array() * x.array();
  }
  else 
    x_filtered <<  single_alpha_ * x_filtered_prev_  + (1-single_alpha_) * x;
  //std::cout << "pre: " << x.transpose()
  //          << " | post: " << x_filtered.transpose()
  //          << "\n";

  x_filtered_prev_ = x_filtered;
}


}

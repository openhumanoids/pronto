// This class implements a torque adjustment suggested
// by IHMC

#include <estimate_tools/torque_adjustment.hpp>
#include <cmath>
#include <stdexcept>

namespace EstimateTools {

TorqueAdjustment::TorqueAdjustment(std::vector<float> k_):k_(k_) {
  // Construct a torque adjustment tool with the given spring constants. The
  // vector k_ should be the same length as the number of efforts and
  // positions (and in the same order), and is measured in units of radians
  // per Newton-meter. To prevent torque adjustment on a joint, set its
  // associated value of k_ to inf (since a joint with no deflection can be
  // thought of as an infinitely stiff spring).
  std::cout << "TorqueAdjustment gains: ";

  for( size_t i=0; i<k_.size(); i++)
    std::cout << k_[i] << ' ';
  std::cout << "\n";

  max_adjustment_ = 0.1; // 0.1 was always used

  position_offset_ = std::vector<float>(k_.size(), 0.0);
}

TorqueAdjustment::TorqueAdjustment(std::vector<float> k_, std::vector<float> position_offset_): k_(k_), position_offset_(position_offset_) {
  // Construct a torque adjustment tool with the given spring constants. The
  // vector k_ should be the same length as the number of efforts and
  // positions (and in the same order), and is measured in units of radians
  // per Newton-meter. To prevent torque adjustment on a joint, set its
  // associated value of k_ to inf (since a joint with no deflection can be
  // thought of as an infinitely stiff spring).
  //
  // In addition, this constructor supports a vector of constant offsets to
  // apply to each joint. Note that these are separate from our encoder
  // offsets and are meant only to handle offsets in the leg joints used for
  // leg odometry, etc.
  std::cout << "TorqueAdjustment gains: ";

  for( size_t i=0; i<k_.size(); i++)
    std::cout << k_[i] << ' ';
  std::cout << "\n";

  std::cout << "Offsets: ";
  for( size_t i=0; i<position_offset_.size(); i++)
    std::cout << position_offset_[i] << ' ';
  std::cout << "\n";

  if (position_offset_.size() != k_.size()) {
    throw std::runtime_error("position_offset_ and k_ should be the same size");
  }

  max_adjustment_ = 0.1; // 0.1 was always used
}


float TorqueAdjustment::magnitudeLimit(float val_in){
  if (val_in > max_adjustment_){
    return max_adjustment_;
  }else if (val_in < -max_adjustment_){
    return -max_adjustment_;
  }
  return val_in;
}

void TorqueAdjustment::processSample(std::vector<float> &position, std::vector<float> &effort ){
  for (size_t i=0; i< k_.size(); i++){
    if (std::isnormal(k_[i])) {
      // Don't do the correction if k_[i] is zero, NaN, or infinite. 
      position[i] -= magnitudeLimit( effort[i] / k_[i]);
      position[i] += position_offset_[i];
    }
  }

  return;

  /*
  // Used for v3:
  //float k_hpz = 7000;
  //float k_x     = 10000;

  // Huge effect - for testing:
  //double k_hpz = 700;
  //double k     = 1000;
  */

}

}

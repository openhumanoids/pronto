#ifndef __TORQUE_ADJUSTMENT_HPP__
#define __TORQUE_ADJUSTMENT_HPP__

#include <iostream>
#include <inttypes.h>
#include <vector>

namespace EstimateTools {

class TorqueAdjustment{
  public:
    TorqueAdjustment(std::vector<std::string> jointsToFilter_, std::vector<float> filterGains_);

    ~TorqueAdjustment(){
    }

    void processSample(std::vector<std::string> names, std::vector<float> &positions, std::vector<float> &efforts);

  private:

    float magnitudeLimit(float val_in);
    float max_adjustment_;

    std::vector<float> filterGains_;
    std::vector<std::string> jointsToFilter_;

};


}

#endif


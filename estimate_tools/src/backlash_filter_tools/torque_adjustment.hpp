#ifndef __TORQUE_ADJUSTMENT_HPP__
#define __TORQUE_ADJUSTMENT_HPP__

#include <iostream>
#include <inttypes.h>
#include <vector>
//#include "atlas/AtlasControlTypes.h"
//#include "atlas/AtlasJointNames.h"

namespace EstimateTools {

class TorqueAdjustment{
  public:
    TorqueAdjustment(std::vector<float> k_);
    TorqueAdjustment(std::vector<float> k_, std::vector<float> offset);

    ~TorqueAdjustment(){
    }

    void processSample(std::vector<float> &position, std::vector<float> &effort );

  private:

    float magnitudeLimit(float val_in);
    float max_adjustment_;

    std::vector<float> k_;
    std::vector<float> offset;

};


}

#endif


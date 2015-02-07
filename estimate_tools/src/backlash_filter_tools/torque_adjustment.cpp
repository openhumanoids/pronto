// This class implements a torque adjustment suggested
// by IHMC

#include <estimate_tools/torque_adjustment.hpp>
//#include "atlas/AtlasJointNames.h"
//#include "atlas/AtlasControlTypes.h"
//#include "atlas/AtlasJointNames.h"

namespace EstimateTools {

TorqueAdjustment::TorqueAdjustment(std::vector<float> k_):k_(k_){
  std::cout << "TorqueAdjustment gains: ";

  for( size_t i=0; i<k_.size(); i++)
    std::cout << k_[i] << ' ';
  std::cout << "\n";

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
  //return;

  // // tmp testing
  // int JOINT_L_LEG_HPZ   = 4;
  // int JOINT_L_LEG_HPX   = 5;
  // int JOINT_L_LEG_HPY   = 6;
  // int JOINT_L_LEG_KNY   = 7;
  // int JOINT_L_LEG_AKY   = 8;
  // int JOINT_L_LEG_AKX   = 9;
  // int JOINT_R_LEG_HPZ   = 10;
  // int JOINT_R_LEG_HPX   = 11;
  // int JOINT_R_LEG_HPY   = 12;
  // int JOINT_R_LEG_KNY   = 13;
  // int JOINT_R_LEG_AKY   = 14;
  // int JOINT_R_LEG_AKX   = 15;


  for (size_t i=0; i< k_.size(); i++){
    position[4+i] -= magnitudeLimit( effort[4+i] / k_[i]);
    position[10+i] -= magnitudeLimit( effort[10+i] / k_[i]);
  }


  return;

  /*
  // Used for v3:
  //float k_hpz = 7000;
  //float k_x     = 10000;

  // Huge effect - for testing:
  //double k_hpz = 700;
  //double k     = 1000;


  position[10] -= magnitudeLimit( effort[10] / k_hpz );
  position[11] -= magnitudeLimit( effort[11] / k_x );
  position[12] -= magnitudeLimit( effort[12] / k_x );

  position[13] -= magnitudeLimit( effort[13] / k_x );
  position[14] -= magnitudeLimit( effort[14] / k_x );
  position[15] -= magnitudeLimit( effort[15] / k_x );

  position[4] -= magnitudeLimit( effort[4] / k_hpz );
  position[5] -= magnitudeLimit( effort[5] / k_x );
  position[6] -= magnitudeLimit( effort[6] / k_x );

  position[7] -= magnitudeLimit( effort[7] / k_x );
  position[8] -= magnitudeLimit( effort[8] / k_x );
  position[9] -= magnitudeLimit( effort[9] / k_x );
  */
}

}

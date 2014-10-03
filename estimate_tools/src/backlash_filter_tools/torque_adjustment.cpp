// This class implements a torque adjustment suggested
// by IHMC

#include <estimate_tools/torque_adjustment.hpp>
//#include "atlas/AtlasJointNames.h"
//#include "atlas/AtlasControlTypes.h"
//#include "atlas/AtlasJointNames.h"

namespace EstimateTools {

TorqueAdjustment::TorqueAdjustment(){

}


double magnitudeLimit(double val_in){
  if (val_in > 0.1){
    return 0.1;
  }else if (val_in < -0.1){
    return -0.1;
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

  // Actual:
  double k_hpz = 7000;
  double k     = 10000;

  // Huge effect - for testing:
  //double k_hpz = 700;
  //double k     = 1000;

  position[10] -= magnitudeLimit( effort[10] / k_hpz );
  position[11] -= magnitudeLimit( effort[11] / k );
  position[12] -= magnitudeLimit( effort[12] / k );

  position[13] -= magnitudeLimit( effort[13] / k );
  position[14] -= magnitudeLimit( effort[14] / k );
  position[15] -= magnitudeLimit( effort[15] / k );

  position[4] -= magnitudeLimit( effort[4] / k_hpz );
  position[5] -= magnitudeLimit( effort[5] / k );
  position[6] -= magnitudeLimit( effort[6] / k );

  position[7] -= magnitudeLimit( effort[7] / k );
  position[8] -= magnitudeLimit( effort[8] / k );
  position[9] -= magnitudeLimit( effort[9] / k );

/*
  position[Atlas::JOINT_R_LEG_HPZ] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPZ] / k_hpz );
  position[Atlas::JOINT_R_LEG_HPX] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPX] / k );
  position[Atlas::JOINT_R_LEG_HPY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPY] / k );

  position[Atlas::JOINT_R_LEG_KNY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_KNY] / k );
  position[Atlas::JOINT_R_LEG_AKY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_AKY] / k );
  position[Atlas::JOINT_R_LEG_AKX] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_AKX] / k );

  position[Atlas::JOINT_L_LEG_HPZ] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPZ] / k_hpz );
  position[Atlas::JOINT_L_LEG_HPX] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPX] / k );
  position[Atlas::JOINT_L_LEG_HPY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPY] / k );

  position[Atlas::JOINT_L_LEG_KNY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_KNY] / k );
  position[Atlas::JOINT_L_LEG_AKY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_AKY] / k );
  position[Atlas::JOINT_L_LEG_AKX] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_AKX] / k );
*/
}

}

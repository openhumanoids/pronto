#include "rbis_yawlock_update.hpp"

namespace MavStateEst {


YawLockHandler::YawLockHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames): 
      lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames)
{
 
  //frames_cpp = new bot::frames(frames);
  lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  model_boost = boost::shared_ptr<ModelClient>(model);

  std::string output_channel = "POSE_YAW_LOCK_OUTPUT";
  int correction_period = 333;

  // should you detect a yaw slip of yaw_slip_threshold_degrees, dont update lock for yaw_slip_disable_period seconds
  bool yaw_slip_detect = true; // was true for atlas
  double yaw_slip_threshold_degrees = 1.5; // degrees
  double yaw_slip_disable_period = 5; //seconds
  std::string behavior_input_mode = "ROBOT_BEHAVIOR";


  yaw_lock_ = new YawLock(lcm_recv, lcm_pub, model_boost);
  yaw_lock_->setParameters(correction_period, yaw_slip_detect, 
    yaw_slip_threshold_degrees, yaw_slip_disable_period );


  if (behavior_input_mode == "CONTROLLER_STATUS"){
    // MIT controller:
    lcm_recv->subscribe( "CONTROLLER_STATUS" ,&YawLockHandler::controllerStatusHandler,this);
  }else if (behavior_input_mode == "ROBOT_BEHAVIOR") {
    // ROBOT_BEHAVIOR comes from IHMC or BDI API
    lcm_recv->subscribe( "ROBOT_BEHAVIOR" ,&YawLockHandler::robotBehaviorHandler,this);
  }else{
    std::cout << "behavior_input_mode not recognised: CONTROLLER_STATUS or ROBOT_BEHAVIOR\n";
    exit(-1);
  }



  Eigen::VectorXd R_scan_match;
    z_indices.resize(1);
    R_scan_match.resize(1);




//if (mode == MODE_YAW) {
    double r_scan_match_yaw = 1.0;//50.0;// = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(0) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(0) = RBIS::chi_ind + 2; // z component only
//  }

  cov_scan_match = R_scan_match.asDiagonal();


}


/// LCM Handlers ////////////////////////////////////
void YawLockHandler::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->state == pronto::controller_status_t::STANDING){
    is_robot_standing = true; 
  }else if (msg->state == pronto::controller_status_t::MANIPULATING){
    is_robot_standing = true; 
  }

  yaw_lock_->setControllerState(is_robot_standing);  
}

void YawLockHandler::robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->behavior == pronto::behavior_t::BEHAVIOR_STAND){
    is_robot_standing = true; 
  }else if (msg->behavior == pronto::behavior_t::BEHAVIOR_MANIPULATE){
    is_robot_standing = true; 
  }

  yaw_lock_->setControllerState(is_robot_standing);  
}



RBISUpdateInterface * YawLockHandler::processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){

  yaw_lock_->setJointState(msg->joint_position, msg->joint_name);



  // Get the Body Position:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << state.position()[0], state.position()[1], state.position()[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(state.quat.w(), state.quat.x(), state.quat.y(), state.quat.z());
  world_to_body.rotate(quat);

  Eigen::Quaterniond world_to_body_quat_correction; 
  if (yaw_lock_->getCorrection(world_to_body, msg->utime, world_to_body_quat_correction) ){

    // send corrections
    std::cout << msg->utime << ": sending POSE_YAW_LOCK\n";
    bot_core::pose_t out;
    out.utime = msg->utime;
    out.pos[0] = 0;
    out.pos[1] = 0;
    out.pos[2] = 0;
    out.orientation[0] = world_to_body_quat_correction.w();
    out.orientation[1] = world_to_body_quat_correction.x();
    out.orientation[2] = world_to_body_quat_correction.y();
    out.orientation[3] = world_to_body_quat_correction.z();
    lcm_pub->publish( "POSE_YAW_LOCK_OUTPUT" , &out);

    Eigen::Vector4d z_meas = Eigen::Vector4d(0,0,0,0); // unused, I believe
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, world_to_body_quat_correction,
        RBISUpdateInterface::yawlock, msg->utime);
  }else{
    return NULL;
  }

}

} // end of namespace
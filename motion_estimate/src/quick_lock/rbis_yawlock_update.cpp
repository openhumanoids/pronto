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

  int correction_period = bot_param_get_int_or_fail(param, "state_estimator.yawlock.correction_period");  
  bool yaw_slip_detect = bot_param_get_boolean_or_fail(param, "state_estimator.yawlock.yaw_slip_detect");  
  double yaw_slip_threshold_degrees = bot_param_get_double_or_fail(param, "state_estimator.yawlock.yaw_slip_threshold_degrees");
  double yaw_slip_disable_period = bot_param_get_double_or_fail(param, "state_estimator.yawlock.yaw_slip_threshold_degrees");
  std::string behavior_channel = bot_param_get_str_or_fail(param, "state_estimator.yawlock.behavior_channel");

  yaw_lock_ = new YawLock(lcm_recv, lcm_pub, model_boost);
  yaw_lock_->setParameters(correction_period, yaw_slip_detect, 
    yaw_slip_threshold_degrees, yaw_slip_disable_period );

  if (behavior_channel == "CONTROLLER_STATUS"){
    // MIT controller:
    lcm_recv->subscribe( "CONTROLLER_STATUS" ,&YawLockHandler::controllerStatusHandler,this);
  }else if (behavior_channel == "ROBOT_BEHAVIOR") {
    // ROBOT_BEHAVIOR comes from IHMC or BDI API
    lcm_recv->subscribe( "ROBOT_BEHAVIOR" ,&YawLockHandler::robotBehaviorHandler,this);
  }else{
    std::cout << "behavior_channel not recognised: CONTROLLER_STATUS or ROBOT_BEHAVIOR\n";
    exit(-1);
  }


  // Bias Estimation
  lcm_recv->subscribe( "IMU_pelvisRearImu" ,&YawLockHandler::insHandler,this);
  char * ins_frame = bot_param_get_str_or_fail(param, "state_estimator.ins.frame");
  bot_frames_get_trans(frames, ins_frame, "body", &ins_to_body);
  free(ins_frame);


  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.yawlock.mode");

  if (strcmp(mode_str, "yawbias") == 0) {
    mode = YawLockMode::MODE_YAWBIAS;
    std::cout << "Yaw lock will estimate yaw rate bias." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = YawLockMode::MODE_YAW;
    std::cout << "Yaw lock will reject yaw drift when standing." << std::endl;
  }
  else if (strcmp(mode_str, "yawbias_yaw") == 0) {
    mode = YawLockMode::MODE_YAWBIAS_YAW;
    std::cout << "Yaw lock will estimate yaw rate bias." << std::endl;
    std::cout << "Yaw lock will reject yaw drift when standing." << std::endl;
  }
  else {
    mode = YawLockMode::MODE_YAW;
    std::cout << "Unrecognized scan matcher mode. Will reject yaw drift by default." << std::endl;
  }
  free(mode_str);


  Eigen::VectorXd R_scan_match;
  if (mode == MODE_YAWBIAS ){
    z_indices.resize(1);
    R_scan_match.resize(1);

    double r_yaw_bias = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw_bias"); // Units deg per sec
    R_scan_match(0) = bot_sq(bot_to_radians(r_yaw_bias)); // Units (rads per sec)^2
    z_indices(0) = RBIS::gyro_bias_ind + 2; // z component only
  }
  else if (mode == MODE_YAW) {
    z_indices.resize(1);
    R_scan_match.resize(1);

    double r_yaw = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw");
    R_scan_match(0) = bot_sq(bot_to_radians(r_yaw));
    z_indices(0) = RBIS::chi_ind + 2; // z component only
  }else if (mode == MODE_YAWBIAS_YAW){
    z_indices.resize(2);
    R_scan_match.resize(2);

    double r_yaw_bias = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw_bias"); // Units deg per sec
    double r_yaw = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw");
    R_scan_match(0) = bot_sq(bot_to_radians(r_yaw_bias)); // Units (rads per sec)^2
    R_scan_match(1) = bot_sq(bot_to_radians(r_yaw));

    // Must by gyro bias followed by chi/yaw
    z_indices(0) = RBIS::gyro_bias_ind + 2; // z component only
    z_indices(1) = RBIS::chi_ind + 2; // z component only
  }
  cov_scan_match = R_scan_match.asDiagonal();


  last_ihmc_walking_utime = 0;
}


/// LCM Handlers ////////////////////////////////////
void YawLockHandler::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg){
  //double body_gyro[3];
  bot_quat_rotate_to(ins_to_body.rot_quat, msg->gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  //if ( yaw_lock_->getIsRobotStanding() ){
  //  std::cout << "INS" << msg->utime << ", " << body_gyro[0]
  //     << ", " << body_gyro[1] << ", " << body_gyro[2] << "\n";
  //}
}

// MIT/Drake status:
void YawLockHandler::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->state == pronto::controller_status_t::STANDING){
    is_robot_standing = true; 
  }else if (msg->state == pronto::controller_status_t::MANIPULATING){
    is_robot_standing = true; // It might be prudent to set this to be false
  }

  yaw_lock_->setIsRobotStanding(is_robot_standing);  
}

// IHMC status:
void YawLockHandler::robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg){
  bool is_robot_standing = false;
  // If standing or manipulating:
  if (msg->behavior == pronto::behavior_t::BEHAVIOR_STAND){
    is_robot_standing = true; 
  }else if (msg->behavior == pronto::behavior_t::BEHAVIOR_MANIPULATE){
    is_robot_standing = true; 
  }

  // NB: The following lines are needed because IHMC's behavior message is faulty
  // It reports the robot standing when it is finishing the last few seconds of a walking plan
  if (msg->behavior == pronto::behavior_t::BEHAVIOR_WALK){
    last_ihmc_walking_utime = msg->utime;
  }
  if (msg->utime - last_ihmc_walking_utime < 3E6){
    //if (is_robot_standing){
    //  std::cout << (msg->utime - last_ihmc_walking_utime) << "... IHMC Not really standing\n";
    //}
    is_robot_standing = false;
  }

  yaw_lock_->setIsRobotStanding(is_robot_standing);  
}

Eigen::Isometry3d getWorldToBody(RBIS state){
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << state.position()[0], state.position()[1], state.position()[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(state.quat.w(), state.quat.x(), state.quat.y(), state.quat.z());
  world_to_body.rotate(quat);
  return world_to_body;
}


RBISUpdateInterface * YawLockHandler::processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){

  // Get the Yaw Rate Bias Estimate:
  Eigen::VectorXd state_measurement = Eigen::VectorXd::Zero(1, 1);
  if ( yaw_lock_->getIsRobotStanding() ){
    state_measurement(0) = body_gyro[2];
  }else{
    state_measurement(0) = state.gyroBias()(2);
  }


  bool yawLockValid = false;
  Eigen::Quaterniond world_to_body_quat_lock; 
  if (mode == MODE_YAW || mode == MODE_YAWBIAS_YAW){  // Get the Yaw estimate:
    yaw_lock_->setJointState(msg->joint_position, msg->joint_name);
    yawLockValid = yaw_lock_->getCorrection( getWorldToBody(state), msg->utime, world_to_body_quat_lock);
  }



  // Create the measurement
  if (mode == MODE_YAWBIAS ){
    return new RBISIndexedMeasurement(z_indices,
        state_measurement, cov_scan_match, RBISUpdateInterface::yawlock,
        msg->utime);
  }
  else if (mode == MODE_YAW) {

    if (yawLockValid){
      return new RBISIndexedPlusOrientationMeasurement(z_indices, state_measurement, cov_scan_match, 
          world_to_body_quat_lock, RBISUpdateInterface::yawlock, msg->utime);
    }else{
      return NULL;
    }

  }else if (mode == MODE_YAWBIAS_YAW){

    if (yawLockValid){
      return new RBISIndexedPlusOrientationMeasurement(z_indices, state_measurement, cov_scan_match, 
          world_to_body_quat_lock, RBISUpdateInterface::yawlock, msg->utime);
    }else{

      // If want to correct but yaw_lock is invalid, only return a bias measurement
      Eigen::VectorXi yawbias_indices(1);
      yawbias_indices(0) = RBIS::gyro_bias_ind + 2;
      Eigen::MatrixXd cov_yaw_bias = Eigen::Matrix<double, 1, 1>::Identity();
      cov_yaw_bias(0) = cov_scan_match(0);

      return new RBISIndexedMeasurement(yawbias_indices,
        state_measurement, cov_yaw_bias, RBISUpdateInterface::yawlock,
        msg->utime);
    }

  }




}



} // end of namespace
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


  // Correction covariance:
  double r_yaw = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw");
  Eigen::VectorXd R_scan_match;
  z_indices.resize(1);
  R_scan_match.resize(1);

  R_scan_match(0) = bot_sq(bot_to_radians(r_yaw));
  z_indices(0) = RBIS::chi_ind + 2; // z component only
  cov_scan_match = R_scan_match.asDiagonal();


  //////////////////////////////////////////////////////
  // Bias Estimation
  lcm_recv->subscribe( "IMU_pelvisRearImu" ,&YawLockHandler::insHandler,this);
  char * ins_frame = bot_param_get_str_or_fail(param, "state_estimator.ins.frame");
  bot_frames_get_trans(frames, ins_frame, "body", &ins_to_body);
  free(ins_frame);

  // Units deg per sec  
  double r_yaw_bias = bot_param_get_double_or_fail(param, "state_estimator.yawlock.r_yaw_bias");
  // Units (rads per sec)^2
  cov_yaw_bias = bot_sq(bot_to_radians(r_yaw_bias));

}

bool is_robot_standing_x;


/// LCM Handlers ////////////////////////////////////
double body_gyro[3];
void YawLockHandler::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg){
  //double body_gyro[3];
  bot_quat_rotate_to(ins_to_body.rot_quat, msg->gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  if (is_robot_standing_x){
    std::cout << "INS" << msg->utime << ", " << body_gyro[0]
       << ", " << body_gyro[1] << ", " << body_gyro[2] << "\n";
  }
}

// MIT/Drake status:
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

// IHMC status:
int64_t last_ihmc_walking_utime = 0;
void YawLockHandler::robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->behavior == pronto::behavior_t::BEHAVIOR_STAND){
    is_robot_standing = true; 
  }else if (msg->behavior == pronto::behavior_t::BEHAVIOR_MANIPULATE){
    is_robot_standing = true; 
  }


  if (msg->behavior == pronto::behavior_t::BEHAVIOR_WALK){
    last_ihmc_walking_utime = msg->utime;
  }

  if (msg->utime - last_ihmc_walking_utime < 3E6){
    if (is_robot_standing){
      std::cout << (msg->utime - last_ihmc_walking_utime) << "NOOOOOOOOOOOOOOOO not really standing\n";
    }

    is_robot_standing = false;
  }


  is_robot_standing_x = is_robot_standing;
  yaw_lock_->setControllerState(is_robot_standing);  
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

  // 
  Eigen::VectorXi index(1);
  index(0) = RBIS::gyro_bias_ind + 2;
  Eigen::MatrixXd measurement_cov = Eigen::Matrix<double, 1, 1>::Identity();
  measurement_cov(0) = cov_yaw_bias ;
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(1, 1);

  if (is_robot_standing_x){
    measurement(0) = body_gyro[2];
  }else{
    measurement(0) = state.gyroBias()(2);
  }


  return new RBISIndexedMeasurement(index,
      measurement, measurement_cov, RBISUpdateInterface::yawlock,
      msg->utime);


  // Get the Body Position:
  yaw_lock_->setJointState(msg->joint_position, msg->joint_name);
  Eigen::Quaterniond world_to_body_quat_lock; 
  bool yawLockValid = yaw_lock_->getCorrection( getWorldToBody(state), msg->utime, world_to_body_quat_lock);


  if (yawLockValid){
    // send corrections
    std::cout << msg->utime << ": sending POSE_YAW_LOCK\n";
    bot_core::pose_t out;
    out.utime = msg->utime;
    out.pos[0] = 0;
    out.pos[1] = 0;
    out.pos[2] = 0;
    out.orientation[0] = world_to_body_quat_lock.w();
    out.orientation[1] = world_to_body_quat_lock.x();
    out.orientation[2] = world_to_body_quat_lock.y();
    out.orientation[3] = world_to_body_quat_lock.z();
    lcm_pub->publish( "POSE_YAW_LOCK_OUTPUT" , &out);

    Eigen::VectorXd z_meas = Eigen::VectorXd::Zero(1, 1);

    //Eigen::Vector4d z_meas = Eigen::Vector4d(0,0,0,0); // unused, I believe
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, world_to_body_quat_lock,
        RBISUpdateInterface::yawlock, msg->utime);
  }else{
    return NULL;
  }
  
}



} // end of namespace
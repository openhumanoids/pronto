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

  lcm_recv->subscribe( "IMU_pelvisRearImu" ,&YawLockHandler::insHandler,this);
  char * ins_frame = bot_param_get_str_or_fail(param, "state_estimator.ins.frame");
  bot_frames_get_trans(frames, ins_frame, "body", &ins_to_body);
  free(ins_frame);

}

bool is_robot_standing_x;


/// LCM Handlers ////////////////////////////////////
void YawLockHandler::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg){
  double body_gyro[3];
  bot_quat_rotate_to(ins_to_body.rot_quat, msg->gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  if (is_robot_standing_x){
    std::cout << "INS" << msg->utime << ", " << body_gyro[0]
       << ", " << body_gyro[1] << ", " << body_gyro[2] << "\n";
  }
}

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

  is_robot_standing_x = is_robot_standing;
  yaw_lock_->setControllerState(is_robot_standing);  
}


RBISUpdateInterface * YawLockHandler::getCorrection(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){
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


RBISUpdateInterface * YawLockHandler::processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){

//  return getCorrection(msg, state, cov);
/*
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;


  Eigen::VectorXd R_scan_match;
  z_indices.resize(1);
  R_scan_match.resize(1);

  R_scan_match(0) = bot_sq(bot_to_radians(r_yaw));
  z_indices(0) = RBIS::chi_ind + 2; // z component only
  cov_scan_match = R_scan_match.asDiagonal();

*/
/*


  double rpyl[3];
  quat_to_euler( Eigen::Quaterniond(laser_pose_msg_.orientation[0], laser_pose_msg_.orientation[1], laser_pose_msg_.orientation[2], laser_pose_msg_.orientation[3]) , rpyl[0],rpyl[1],rpyl[2]);


  out.measured_dim = 4;
  out.z_indices = {8,9,10,11}; // yaw, x, y, z
  //msg->state[8]
  // out.z_effective = { rpyl[2] , msg->state[9], msg->state[10], msg->state[11]};
  out.z_effective = { msg->state[8] , msg->state[9], msg->state[10], msg->state[11]};
  out.measured_cov_dim = 16;
  out.R_effective = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,};
  out.R_effective[0]  = cl_cfg_.yaw_cov; // yaw
  out.R_effective[5]  = cl_cfg_.xyz_cov; // x
  out.R_effective[10] = cl_cfg_.xyz_cov; // y
  out.R_effective[15] = cl_cfg_.xyz_cov; // z

*/

  Eigen::VectorXi index(4);
  index(0) = 8;
  index(1) = 9;
  index(2) = 10;
  index(3) = 11;


  Eigen::VectorXd measurement;
  measurement = Eigen::VectorXd::Zero(4, 1);// {0.0,1.0,2.0,3.0};
  measurement(0) = 0.0;
  measurement(1) = 0.0;
  measurement(2) = 0.0;
  measurement(3) = 0.0;

  Eigen::MatrixXd measurement_cov = 1. * Eigen::Matrix<double, 3, 3>::Identity();
  measurement_cov(0) = 1000;
  measurement_cov(5) = 1000;
  measurement_cov(10) = 1000;
  measurement_cov(15) = 1000;

/*
  RBISIndexedMeasurement(const Eigen::VectorXi & index_, const Eigen::VectorXd & measurement_,
      const Eigen::MatrixXd & measurement_cov_, RBISUpdateInterface::sensor_enum sensor_id_, int64_t utime) :
      RBISUpdateInterface(sensor_id_, utime), index(index_), measurement(measurement_), measurement_cov(
          measurement_cov_)
          */  

  std::cout << "fuck off\n";
  return new RBISIndexedMeasurement(index,
      measurement,
      measurement_cov,
      RBISUpdateInterface::yawlock, msg->utime);


}

} // end of namespace
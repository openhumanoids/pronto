#include "rbis_yaw_lock_update.hpp"

namespace MavStateEst {


YawLockHandler::YawLockHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames): 
      lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames)
{
 
  //frames_cpp = new bot::frames(frames);
  lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  model_boost = boost::shared_ptr<ModelClient>(model);

  yaw_lock_ = new YawLock(lcm_, model_boost);
  yaw_lock_->setParameters(cl_cfg_.correction_period, cl_cfg_.yaw_slip_detect, 
    cl_cfg_.yaw_slip_threshold_degrees, cl_cfg_.yaw_slip_disable_period );

}



/// LCM Handlers ////////////////////////////////////
void YawLockHandler::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  yaw_lock_->setControllerState(msg->state);  
}

void YawLockHandler::forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg){
  force_torque_ = *msg;
  force_torque_init_ = true;


  // F/T isn't received via a processMessage handler, so this republishes it here:
  if (republish_sensors_)
    lcm_pub->publish(channel, msg);

}


RBISUpdateInterface * YawLockHandler::processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){

  yaw_lock_->setJointState(msg->joint_position, msg->joint_name);



  // Get the Body Position:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1],
                                               msg->orientation[2], msg->orientation[3]);
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
    lcm_->publish(cl_cfg_.output_channel , &out);

  }

  BotTrans odo_deltaT = getPoseAsBotTrans(odo_delta);
  BotTrans odo_positionT = getPoseAsBotTrans(odo_position);
  if (publish_diagnostics_) sendTransAsVelocityPose(odo_deltaT, utime, prev_utime, "POSE_BODY_LEGODO_VELOCITY");

  return leg_odo_common_->createMeasurement(odo_positionT, odo_deltaT,
                                            utime, prev_utime,
                                            odo_position_status, odo_delta_status);
}
#include "rbis_legodo_update.hpp"
#include <path_util/path_util.h>
#include <string>
#include <math.h>


using namespace std;

namespace MavStateEst {


LegOdoHandler::LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames): 
      lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames)
{
  std::cout << "LegOdo will compute directly, in thread\n";
  verbose_ =2; // 3 lots, 2 some, 1 v.important
  
  //frames_cpp = new bot::frames(frames);
  lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  model_boost = boost::shared_ptr<ModelClient>(model);
  
  leg_est_ = new leg_estimate(lcm_pub_boost, param, model_boost );
  leg_odo_common_ = new LegOdoCommon(lcm_recv, lcm_pub, param);

  use_torque_adjustment_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.torque_adjustment");
  if (use_torque_adjustment_){
    std::cout << "Torque-based joint angle adjustment: Using\n";

    int n_gains = bot_param_get_array_len (param, "state_estimator.legodo.adjustment_gain");
    double gains_in[n_gains];
    bot_param_get_double_array_or_fail(param, "state_estimator.legodo.adjustment_gain", &gains_in[0], n_gains);
    std::vector<float> k;
    for (int i =0; i < n_gains;i++){
      k.push_back( (float) gains_in[i] );
    }

    std::vector<std::string> jnames_v;
    char** jnames = bot_param_get_str_array_alloc(param, "state_estimator.legodo.adjustment_joints");
    if (jnames == NULL) {
      fprintf(stderr, "Error: must specify state_estimator.legodo.adjustment_joints\n");
      exit(1);
    }
    else {
      for (int i = 0; jnames[i]; i++) {
        jnames_v.push_back(std::string(jnames[i]));
      }
    }
    bot_param_str_array_free(jnames);

    torque_adjustment_ = new EstimateTools::TorqueAdjustment(jnames_v, k);

  }else{
    std::cout << "Torque-based joint angle adjustment: Not Using\n";
  }

  zero_initial_velocity = bot_param_get_int_or_fail(param, "state_estimator.legodo.zero_initial_velocity");
  std::cout << "Will assume zero kinematic velocity for first " <<  zero_initial_velocity << " tics\n";

  // Non-algoritim Settings
  publish_diagnostics_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.publish_diagnostics");  
  republish_incoming_poses_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.republish_incoming_poses");  
  bool republish_cameras = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.republish_cameras");    
  
  if ( (lcm_pub != lcm_recv) && republish_incoming_poses_) { 
    republish_incoming_poses_ = true; // Only republish if the lcm objects are different (same logic as for main app)
  }else{
    republish_incoming_poses_ = false;
  }
  if (republish_incoming_poses_){
    std::cout << "Will republish a variety of channels\n";
    lcm_recv->subscribe("VICON_BODY|VICON_FRONTPLATE|VICON_pelvis_val",&LegOdoHandler::viconHandler,this);
  }else{
    std::cout << "Will not republish other data\n";
  }
  
  channel_force_torque = bot_param_get_str_or_fail(param, "state_estimator.legodo.channel_force_torque");
  lcm_recv->subscribe(channel_force_torque.c_str(),&LegOdoHandler::forceTorqueHandler,this);
  std::cout << "Subscribing to "<< channel_force_torque <<" for Force/Torque measurements\n";
  // Note reuse of config variable:
  republish_sensors_ = bot_param_get_boolean_or_fail(param, "state_estimator.republish_sensors");
  
  // Arbitrary Subscriptions:
  if (lcm_pub != lcm_recv && republish_cameras) {
    std::cout << "Will republish camera data\n";
    lcm_recv->subscribe("WEBCAM|POSE_BODY_ALT",&LegOdoHandler::republishHandler,this);  
  }
 
  lcm_recv->subscribe("CONTROLLER_FOOT_CONTACT",&LegOdoHandler::controllerInputHandler,this);
  n_control_contacts_left_ = -1;
  n_control_contacts_right_ = -1;


  prev_worldvicon_to_body_vicon_.setIdentity();
  prev_vicon_utime_ = -1;
  
  force_torque_init_ = false;

  left_force_tare_ = bot_param_get_double_or_fail(param, "state_estimator.legodo.left_force_tare");
  right_force_tare_ = bot_param_get_double_or_fail(param, "state_estimator.legodo.right_force_tare");
  
}

/// Extra-class Functions  /////////////////////////////
int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  
  return status;
}

BotTrans getPoseAsBotTrans(Eigen::Isometry3d odo_delta){
  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
  Eigen::Vector3d motion_T = odo_delta.translation();
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(odo_delta.rotation());
  msgT.trans_vec[0] = motion_T(0);
  msgT.trans_vec[1] = motion_T(1);
  msgT.trans_vec[2] = motion_T(2);
  msgT.rot_quat[0] = motion_R.w();
  msgT.rot_quat[1] = motion_R.x();
  msgT.rot_quat[2] = motion_R.y();
  msgT.rot_quat[3] = motion_R.z();
  
  return msgT;
}

// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose (for visualization in signal scope)
bot_core::pose_t getBotTransAsBotPoseVelocity(BotTrans bt, int64_t utime ){
  bot_core::pose_t pose;
  pose.utime = utime;
  pose.pos[0] = 0;
  pose.pos[1] = 0;
  pose.pos[2] = 0;
  pose.orientation[0] = 0;
  pose.orientation[1] = 0;
  pose.orientation[2] = 0;
  pose.orientation[3] = 0;
  pose.vel[0] = bt.trans_vec[0];
  pose.vel[1] = bt.trans_vec[1];
  pose.vel[2] = bt.trans_vec[2];
  pose.rotation_rate[0] = 0;//bt.rot_quat[0];
  pose.rotation_rate[1] = 0;//bt.rot_quat[1];
  pose.rotation_rate[2] = 0;//bt.rot_quat[2];
  return pose;
}

// TODO: learn how to directly copy the underlying data
bot_core::pose_t getPoseAsBotPoseFull(PoseT pose){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   pose.utime;
  pose_msg.pos[0] = pose.pos[0];
  pose_msg.pos[1] = pose.pos[1];
  pose_msg.pos[2] = pose.pos[2];  
  pose_msg.orientation[0] =  pose.orientation[0];  
  pose_msg.orientation[1] =  pose.orientation[1];  
  pose_msg.orientation[2] =  pose.orientation[2];  
  pose_msg.orientation[3] =  pose.orientation[3];
  
  pose_msg.vel[0] = pose.vel[0];
  pose_msg.vel[1] = pose.vel[1];
  pose_msg.vel[2] = pose.vel[2];
  pose_msg.rotation_rate[0] = pose.rotation_rate[0];
  pose_msg.rotation_rate[1] = pose.rotation_rate[1];
  pose_msg.rotation_rate[2] = pose.rotation_rate[2];  
  
  pose_msg.accel[0] = pose.accel[0];
  pose_msg.accel[1] = pose.accel[1];
  pose_msg.accel[2] = pose.accel[2];
  return pose_msg;
}

Eigen::Isometry3d getPoseAsIsometry3d(PoseT pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << pose.pos[0], pose.pos[1] , pose.pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose.orientation[0], pose.orientation[1], 
                                               pose.orientation[2], pose.orientation[3]);
  pose_iso.rotate(quat);
  return pose_iso;
}

/// LCM Handlers ////////////////////////////////////
void LegOdoHandler::controllerInputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_foot_contact_t* msg){
  n_control_contacts_left_ = msg->num_left_foot_contacts;
  n_control_contacts_right_ = msg->num_right_foot_contacts;
}

void LegOdoHandler::forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg){
  force_torque_ = *msg;
  force_torque_init_ = true;

  // F/T isn't received via a processMessage handler, so this republishes it here:
  if (republish_sensors_)
    lcm_pub->publish(channel, msg);

}


RBISUpdateInterface * LegOdoHandler::processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov){

  if (!force_torque_init_){
    std::cout << "Force/Torque message not received yet, not integrating leg odometry =========================\n";
    return NULL;    
  } 

  world_to_body_full_.utime = state.utime;
  world_to_body_full_.pos = Eigen::Vector3d( state.position()[0], state.position()[1], state.position()[2] );
  world_to_body_full_.vel = Eigen::Vector3d( state.velocity()[0], state.velocity()[1], state.velocity()[2] );
  world_to_body_full_.orientation = Eigen::Vector4d( state.quat.w(), state.quat.x(), state.quat.y(), state.quat.z() );
  world_to_body_full_.rotation_rate = Eigen::Vector3d( state.angularVelocity()[0], state.angularVelocity()[1], state.angularVelocity()[2]);
  world_to_body_full_.accel = Eigen::Vector3d( state.acceleration()[0], state.acceleration()[1], state.acceleration()[2] );
  leg_est_->setPoseBody(  getPoseAsIsometry3d(world_to_body_full_)      );

  // NB: remove a tare value from each sensor. This is temporary for Valkyrie in May 2016 until NASA enable taring
  float left_force_corrected = force_torque_.sensors[0].force[2] - left_force_tare_;
  float right_force_corrected = force_torque_.sensors[1].force[2] - right_force_tare_;

  // 1. Do the Leg Odometry Integration
  leg_est_->setFootSensing(  FootSensing( fabs( left_force_corrected ), force_torque_.sensors[0].moment[0],  force_torque_.sensors[0].moment[1]),
                             FootSensing( fabs( right_force_corrected ), force_torque_.sensors[1].moment[0],  force_torque_.sensors[1].moment[1]));
  //leg_est_->setFootSensing(  FootSensing( fabs(force_torque_.sensors[0].force[2]), force_torque_.sensors[0].moment[0],  force_torque_.sensors[0].moment[1]),
  //                           FootSensing( fabs(force_torque_.sensors[1].force[2]), force_torque_.sensors[1].moment[0],  force_torque_.sensors[1].moment[1]));
  leg_est_->setControlContacts(n_control_contacts_left_, n_control_contacts_right_);

  // 1.1 Apply the joint torque-to-angle adjustment
  // TODO: this should probably be done inside the leg_est class and not here
  std::vector <float> mod_position, mod_effort;
  mod_position = msg->joint_position;
  mod_effort = msg->joint_effort;
  if (use_torque_adjustment_){
    torque_adjustment_->processSample(msg->joint_name, mod_position, mod_effort );
  }

  float odo_delta_status = leg_est_->updateOdometry(msg->joint_name, mod_position,
                                                    msg->joint_velocity, msg->utime);
  if (odo_delta_status<0){
    if (verbose_ >= 3) std::cout << "Leg Odometry is not valid not integrating =========================\n";
    
    if(publish_diagnostics_){
      Eigen::Isometry3d odo_delta;
      int64_t utime, prev_utime;
      leg_est_->getLegOdometryDelta(odo_delta, utime, prev_utime);
      
      BotTrans odo_deltaT = getPoseAsBotTrans(odo_delta);
      sendTransAsVelocityPose( odo_deltaT, utime, prev_utime, "POSE_BODY_LEGODO_VELOCITY_FAIL");
    }
    
    return NULL;
  }
  
  // 2. If successful make a RBIS Measurement
  Eigen::Isometry3d odo_delta, odo_position;
  int64_t utime, prev_utime;
  leg_est_->getLegOdometryDelta(odo_delta, utime, prev_utime);
  int64_t temp;
  bool odo_position_status = leg_est_->getLegOdometryWorldConstraint(odo_position,temp);
  
  // Ignore the calculated velocity at launch:
  zero_initial_velocity--;
  if (zero_initial_velocity > 0){
    odo_delta.setIdentity();
    odo_position.setIdentity();
  }


  BotTrans odo_deltaT = getPoseAsBotTrans(odo_delta);
  BotTrans odo_positionT = getPoseAsBotTrans(odo_position);
  if (publish_diagnostics_) sendTransAsVelocityPose(odo_deltaT, utime, prev_utime, "POSE_BODY_LEGODO_VELOCITY");

  return leg_odo_common_->createMeasurement(odo_positionT, odo_deltaT,
                                            utime, prev_utime,
                                            odo_position_status, odo_delta_status);

}

void LegOdoHandler::republishHandler (const lcm::ReceiveBuffer* rbuf, const std::string& channel){
  lcm_pub->publish(channel, rbuf->data, rbuf->data_size);
}

void LegOdoHandler::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  Eigen::Isometry3d worldvicon_to_frontplate_vicon;
  worldvicon_to_frontplate_vicon.setIdentity();
  worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
  worldvicon_to_frontplate_vicon.rotate(quat); 
  
  // Apply the body to frontplate transform
  Eigen::Isometry3d frontplate_vicon_to_body_vicon;
  // Atlas:
  //get_trans_with_utime(frames, "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);        
  get_trans_with_utime(frames, "body_vicon" , "vicon_frame", msg->utime, frontplate_vicon_to_body_vicon);          
  Eigen::Isometry3d worldvicon_to_body_vicon = worldvicon_to_frontplate_vicon* frontplate_vicon_to_body_vicon;
  bot_core::pose_t pose_msg = getPoseAsBotPose(worldvicon_to_body_vicon, msg->utime);
  
  // determine the vicon body frame velocity:
  if (prev_vicon_utime_ > 0){
    // the delta transform between the previous and current 
    Eigen::Isometry3d delta_vicon = prev_worldvicon_to_body_vicon_.inverse() * worldvicon_to_body_vicon;
    double elapsed_time = ( (double) msg->utime -  prev_vicon_utime_)/1000000;
    pose_msg.vel[0] = delta_vicon.translation().x() / elapsed_time;
    pose_msg.vel[1] = delta_vicon.translation().y() / elapsed_time;
    pose_msg.vel[2] = delta_vicon.translation().z() / elapsed_time;
  }
  
  lcm_pub->publish("POSE_VICON", &pose_msg );    
  
  prev_worldvicon_to_body_vicon_ = worldvicon_to_body_vicon;
  prev_vicon_utime_ = msg->utime;
}

/// Publishing Functions 
// Convert the delta position into a velocity 
// as a bot_pose message for visualization with signal scope:
void LegOdoHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = leg_odo_common_->getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}

} // end of namespace

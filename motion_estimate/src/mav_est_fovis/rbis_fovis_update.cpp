#include "rbis_fovis_update.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {

FovisHandler::FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
                           BotParam * param, BotFrames * frames): lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames){
  verbose_ = true;
  publish_diagnostics_ = false;
  
  pc_vis_ = new pronto_vis( lcm_pub->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(7000,"Pronto-VO Pose t0",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(7001,"Pronto-VO Pose t1 kin",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(7002,"Pronto-VO Pose t1 vo",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(7003,"Pronto-VO Pose current",5,1) );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(7010,"Pronto-VO Pose t0 internal",5,1) );

  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.fovis.mode");

  if (strcmp(mode_str, "lin_rot_rate") == 0) {
    mode = FovisHandler::MODE_LIN_AND_ROT_RATE;
    std::cout << "FOVIS will provide velocity and rotation rates." << std::endl;
  }
  else if (strcmp(mode_str, "lin_rate") == 0){
    mode = FovisHandler::MODE_LIN_RATE;
    std::cout << "FOVIS will provide velocity rates." << std::endl;
  }
  else if (strcmp(mode_str, "lin") == 0){
    mode = FovisHandler::MODE_LIN;
    std::cout << "FOVIS will provide linear position corrections." << std::endl;
  }
  else{

    // ... incomplete...
  }
  free(mode_str);
  
  Eigen::VectorXd R_fovis;
  if (mode == MODE_LIN_AND_ROT_RATE) {
    z_indices.resize(6);
    R_fovis.resize(6);
  }
  else if (mode == MODE_LIN_RATE){
    z_indices.resize(3);
    R_fovis.resize(3);
  }
  else if (mode == MODE_LIN){
    z_indices.resize(3);
    R_fovis.resize(3);
  }
  else{
    // ... incomplete...
  }

  // Initialize covariance matrix based on mode.
  if (mode == MODE_LIN_AND_ROT_RATE) {
    double R_fovis_vxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vxyz");
    double R_fovis_vang = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vang");
    R_fovis(0) = bot_sq(R_fovis_vxyz);
    R_fovis(1) = bot_sq(R_fovis_vxyz);
    R_fovis(2) = bot_sq(R_fovis_vxyz);
    R_fovis(3) = bot_sq(R_fovis_vang);
    R_fovis(4) = bot_sq(R_fovis_vang);
    R_fovis(5) = bot_sq(R_fovis_vang);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
    z_indices.tail<3>() = eigen_utils::RigidBodyState::angularVelocityInds();

  }else if (mode == MODE_LIN_RATE){
    double R_fovis_vxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vxyz");
    R_fovis(0) = bot_sq(R_fovis_vxyz);
    R_fovis(1) = bot_sq(R_fovis_vxyz);
    R_fovis(2) = bot_sq(R_fovis_vxyz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();

  }else if (mode == MODE_LIN){
    double R_fovis_pxy = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_pxy");
    R_fovis(0) = bot_sq(R_fovis_pxy);
    R_fovis(1) = bot_sq(R_fovis_pxy);
    double R_fovis_pz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_pz");
    R_fovis(2) = bot_sq(R_fovis_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();

  }else{
    // ..incomplete
  }

  cov_fovis = R_fovis.asDiagonal();

  prev_t0_body_ = Eigen::Isometry3d::Identity();
  prev_t0_body_internal_ = Eigen::Isometry3d::Identity();
  prev_t0_body_utime_ = 0;

}


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

// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose 
// [duplicated in rbis_legodo_common.cpp]
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



RBISUpdateInterface * FovisHandler::processMessage(const pronto::update_t * msg, MavStateEstimator* state_estimator){

  Eigen::Isometry3d t0_body = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t0_body_internal = Eigen::Isometry3d::Identity();
  if (msg->prev_timestamp != prev_t0_body_utime_){
    // TODO check that these trans are valid
    int status = get_trans_with_utime(frames, "body" , "local", msg->prev_timestamp, t0_body);


    /////////////////////////////////////////////////////////////////////////////////
    //updateHistory::historyMapIterator prev_it = state_estimator->history.updateMap.find(msg->prev_timestamp);
    updateHistory::historyMapIterator lower_it = state_estimator->history.updateMap.lower_bound(msg->prev_timestamp);
    double diff_utime = ( lower_it->first - msg->prev_timestamp ) *1E-6;
    if (diff_utime > 0.025){
      std::cout << "FOIVS: time difference for VO delta root pose is too great ("<< diff_utime <<"sec). Will not use\n";
      return NULL;
    }


    RBISUpdateInterface * t0_body_RBISInterface = lower_it->second;
    RBIS t0_body_RBIS = t0_body_RBISInterface->posterior_state;
    t0_body_internal.translation() = Eigen::Vector3d( t0_body_RBIS.position()[0], t0_body_RBIS.position()[1], t0_body_RBIS.position()[2] );
    t0_body_internal.rotate( Eigen::Quaterniond( t0_body_RBIS.quat.w(), t0_body_RBIS.quat.x(), t0_body_RBIS.quat.y(), t0_body_RBIS.quat.z()) );
    /////////////////////////////////////////////////////////////////////////////////

    prev_t0_body_ = t0_body;
    prev_t0_body_internal_ = t0_body_internal;
    prev_t0_body_utime_ = msg->prev_timestamp;


  }else{
    t0_body = prev_t0_body_;
    t0_body_internal = prev_t0_body_internal_;
  }


  Eigen::Isometry3d t1_body = Eigen::Isometry3d::Identity();
  get_trans_with_utime(frames, "body" , "local", msg->timestamp, t1_body);

  Eigen::Isometry3d t0t1_body_vo = Eigen::Isometry3d::Identity();
  t0t1_body_vo.translation() = Eigen::Vector3d( msg->translation[0], msg->translation[1], msg->translation[2] );
  t0t1_body_vo.rotate( Eigen::Quaterniond( msg->rotation[0], msg->rotation[1], msg->rotation[2], msg->rotation[3] ) );

  Eigen::Isometry3d t1_body_vo = t0_body_internal * t0t1_body_vo; // the pose of the robot as estimated by applying the VO translation on top of t0 state

  if (publish_diagnostics_){

    Isometry3dTime t0_body_T = Isometry3dTime(msg->prev_timestamp , t0_body );
    pc_vis_->pose_to_lcm_from_list(7000, t0_body_T );
    Isometry3dTime t1_body_T = Isometry3dTime(msg->timestamp , t1_body );
    pc_vis_->pose_to_lcm_from_list(7001, t1_body_T );


    Isometry3dTime t1_body_vo_T = Isometry3dTime(msg->timestamp , t1_body_vo );
    pc_vis_->pose_to_lcm_from_list(7002, t1_body_vo_T );


    Isometry3dTime t0_body_internal_T = Isometry3dTime(msg->prev_timestamp , t0_body_internal );
    pc_vis_->pose_to_lcm_from_list(7010, t0_body_internal_T );



    RBIS head_state;
    RBIM head_cov;
    state_estimator->getHeadState(head_state, head_cov);

    Eigen::Isometry3d current_body;
    current_body.setIdentity();
    current_body.translation() = Eigen::Vector3d( head_state.position()[0], head_state.position()[1], head_state.position()[2] );
    current_body.rotate( Eigen::Quaterniond( head_state.quat.w(), head_state.quat.x(), head_state.quat.y(), head_state.quat.z() ) );
    Isometry3dTime current_body_T = Isometry3dTime(msg->timestamp , current_body );
    pc_vis_->pose_to_lcm_from_list(7003, current_body_T );

    //std::cout << t1_body.translation().transpose() << " body\n";
    //std::cout << t1_body_vo.translation().transpose() << " vo delta\n";
    //Eigen::Vector3d diff = Eigen::Vector3d( t1_body_vo.translation() - t1_body.translation() ); 
    //std::cout << diff.transpose() << " is diff\n\n";
  }

  if (msg->estimate_status == pronto::update_t::ESTIMATE_VALID){
  }else{
    std::cout << "FovisHandler: FOVIS failure, not integrating this measurement\n";
    return NULL;
  }
  
  // TODO: explore why this is allowed to be published upstream
  if ( isnan( msg->translation[0]) ){
    std::cout << "FovisHandler: FOVIS produced NaN. x="<< msg->translation[0] << ", quitting\n";
    exit(-1); // keep this exit until we have found the source of the NaN in Fovis
    return NULL;
  }


  
  BotTrans odo_deltaT;
  memset(&odo_deltaT, 0, sizeof(odo_deltaT));
  memcpy(odo_deltaT.trans_vec, msg->translation, 3 * sizeof(double));
  memcpy(odo_deltaT.rot_quat,  msg->rotation   , 4 * sizeof(double));

  BotTrans odo_velT = getTransAsVelocityTrans(odo_deltaT, msg->timestamp, msg->prev_timestamp);
  
  if (publish_diagnostics_){
    // Get the velocity as a pose message:
    bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(odo_velT, msg->timestamp)  ;
    lcm_pub->publish("POSE_BODY_FOVIS_VELOCITY", &vel_pose );      
  }
  
  if (mode == MODE_LIN_AND_ROT_RATE) { // Working on this:
    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, odo_velT.rot_quat);
    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(odo_velT.trans_vec);
    //  eigen_utils::botDoubleToQuaternion(quat, msg->quat);
    //  z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->trans);
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_fovis, quat, RBISUpdateInterface::fovis,
            msg->timestamp);

  }else if (mode == MODE_LIN_RATE) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>( odo_velT.trans_vec ), cov_fovis, RBISUpdateInterface::fovis,
        msg->timestamp);

  }else if (mode == MODE_LIN) {
    Eigen::VectorXd z_meas(3);
    z_meas.head<3>() = Eigen::Vector3d( t1_body_vo.translation().x()  , t1_body_vo.translation().y() , t1_body_vo.translation().z() );
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
        z_meas, cov_fovis, RBISUpdateInterface::fovis,
        msg->timestamp);

  }else{
    std::cout << "FovisHandler Mode not supported, exiting\n";
    return NULL;
  }
  
}


/// Everything below is duplicated in rbis_legodo_common.cpp
void printTrans(BotTrans bt, std::string message){
  std::cout << message << ": "
      << bt.trans_vec[0] << ", " << bt.trans_vec[1] << ", " << bt.trans_vec[2] << " | "
      << bt.rot_quat[0] << ", " << bt.rot_quat[1] << ", " << bt.rot_quat[2] << ", " << bt.rot_quat[3] << "\n";
}


// Difference the transform and scale by elapsed time:
BotTrans FovisHandler::getTransAsVelocityTrans(BotTrans msgT, int64_t utime, int64_t prev_utime){
  Eigen::Isometry3d msgE = pronto::getBotTransAsEigen(msgT);
  Eigen::Isometry3d msgE_vel = pronto::getDeltaAsVelocity(msgE, (utime-prev_utime) );
  BotTrans msgT_vel;
  msgT_vel = pronto::getEigenAsBotTrans(msgE_vel);
 
  return msgT_vel;
}


/// Publishing Functions 
// Convert the delta position into a velocity 
// as a bot_pose message for visualization with signal scope:
void FovisHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}

} // end of namespace

#include "LegOdoWrapper.hpp"

App::App(boost::shared_ptr<lcm::LCM> &lcm_subscribe_,  boost::shared_ptr<lcm::LCM> &lcm_publish_, CommandLineConfig& cl_cfg_) : 
        LegOdoWrapper(lcm_subscribe_, lcm_publish_, cl_cfg_) {

  setupLegOdo();

  lcm_subscribe_->subscribe("FORCE_TORQUE",&App::forceTorqueHandler,this);
  lcm_subscribe_->subscribe("ATLAS_STATE",&App::jointStateHandler,this);
  if ( cl_cfg_.republish_incoming){
    lcm_subscribe_->subscribe("VICON_BODY|VICON_FRONTPLATE",&App::viconHandler,this);
  }

  force_torque_init_ = false;
}

App::~App() {
  delete(leg_est_);
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

void LegOdoWrapper::setupLegOdo() {
  if (cl_cfg_.param_file == ""){
    botparam_ = bot_param_new_from_server(lcm_subscribe_->getUnderlyingLCM(), 0);
  }else{
    //std::string param_file = "drc_robot_02.cfg";
    std::string param_file_full = std::string(getConfigPath()) +'/' + std::string(cl_cfg_.param_file);
    botparam_ = bot_param_new_from_file(param_file_full.c_str());
  }
  
  // TODO: not sure what do do here ... what if i want the frames from the file?
  //frames_ = bot_frames_new(NULL, botparam_);
  frames_ = bot_frames_get_global(lcm_subscribe_->getUnderlyingLCM(), botparam_);
  // frames_cpp_ = new bot::frames(frames_);

  if (cl_cfg_.urdf_file == ""){
    model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_subscribe_->getUnderlyingLCM(), 0));
  }else{
    // was:
    //std::string urdf_file_full = std::string(getModelsPath()) +"/mit_gazebo_models/mit_robot/" + std::string(cl_cfg_.urdf_file);
    std::string urdf_file_full = std::string(getModelsPath()) +"/mit_robot/" + std::string(cl_cfg_.urdf_file);
    model_ = boost::shared_ptr<ModelClient>(new ModelClient( urdf_file_full  ));
  }

  leg_est_ = new leg_estimate(lcm_publish_, botparam_, model_);
}

void App::forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg){
  force_torque_ = *msg;
  force_torque_init_ = true;
}

void App::jointStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  if ( cl_cfg_.begin_timestamp > -1){
    if (msg->utime <  cl_cfg_.begin_timestamp ){
      double seek_seconds = (cl_cfg_.begin_timestamp - msg->utime)*1E-6;
      std::cout << msg->utime << " too early | seeking " << seek_seconds    << "secs, to " << cl_cfg_.begin_timestamp << "\n";
      return;
    }
  }
  if ( cl_cfg_.end_timestamp > -1){
    if (msg->utime >  cl_cfg_.end_timestamp ){
      //terminate();
      std::cout << msg->utime << " finishing\n";
      exit(-1);
      return;
    }
  }

  if (!force_torque_init_){
    std::cout << "FORCE_TORQUE not received yet, not integrating leg odometry =========================\n";
    return;
  }

  leg_est_->setFootSensing(  FootSensing( fabs(force_torque_.sensors[0].force[2]), force_torque_.sensors[0].moment[0],  force_torque_.sensors[0].moment[1]),
                             FootSensing( fabs(force_torque_.sensors[1].force[2]), force_torque_.sensors[1].moment[0],  force_torque_.sensors[1].moment[1]));
  leg_est_->updateOdometry(msg->joint_name, msg->joint_position, msg->joint_velocity, msg->utime);

  Eigen::Isometry3d world_to_body = leg_est_->getRunningEstimate();
  bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body, msg->utime);
  lcm_publish_->publish("POSE_BODY", &pose_msg );
}


void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){

  Eigen::Isometry3d worldvicon_to_frontplate_vicon;
  worldvicon_to_frontplate_vicon.setIdentity();
  worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1],
                                               msg->quat[2], msg->quat[3]);
  worldvicon_to_frontplate_vicon.rotate(quat);

  // Apply the body to frontplate transform
  Eigen::Isometry3d frontplate_vicon_to_body_vicon;
  // frames_cpp_->get_trans_with_utime( "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);
  get_trans_with_utime(frames_, "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);


  Eigen::Isometry3d worldvicon_to_body_vicon = worldvicon_to_frontplate_vicon* frontplate_vicon_to_body_vicon;

  bot_core::pose_t pose_msg = getPoseAsBotPose(worldvicon_to_body_vicon, msg->utime);
  lcm_publish_->publish("POSE_VICON", &pose_msg );
}

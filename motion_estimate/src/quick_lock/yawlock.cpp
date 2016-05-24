// if transitioned to standing, capture yaw
// there after, issue this yaw as a correction to 
// to the state estimate

/*

inputs are:
- core robot state
- pose body
- controller status

when standing or manipulating
- and haven't seen a slip recently determine correction (at about 1Hz)

using current state (pose body and joints):
- find pose of each foot

have we just started standing:
- capture the positions of the feet
  [return]

if a yaw jump is detected disable for 5 seconds
- this is done by comparing the average yaw of the feet
  not to earlier. it shouldn't have changed
- to avoid closed loop feedback on controller

else:

assuming the two feet havent slipped since standing
- get reverse FK from each foot to pelvis
- average to give pelvis orientation
- only use the yaw element in the state correction


TODO: look at atlas logs and see if yaw slip detection
occurred during the finals
TODO: one piece of info we dont currently use is the 
relative foot velocity in either estimation or failure
detection
*/

#include "yawlock.hpp"




YawLock::YawLock(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, boost::shared_ptr<ModelClient> &model_):
    lcm_recv(lcm_recv), lcm_pub(lcm_pub), model_(model_), is_robot_standing_(false){
  // Controller state set to unknonn at start

  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    exit(-1);
  }

  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  joint_angles_init_ = false;
  lock_init_ = false;
  utime_disable_until_ = 0; // don't disable at start (but will be uninitialized)

  counter_=0;

}


Eigen::Isometry3d KDLToEigen(KDL::Frame tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}

bool YawLock::getCorrection(Eigen::Isometry3d world_to_body, 
	int64_t body_utime, Eigen::Quaterniond &world_to_body_quat_correction){

  // Occasionally send a correction when in either standing or manipulating mode
  if (counter_ % correction_period_ != 0){ // Correct the yaw drift every X tics
    counter_++;
    return false;
  }else{
    counter_++;
  }
  if (!is_robot_standing_){
    std::cout << body_utime << " not in standing or manipulation mode, not correcting\n";
    lock_init_ = false;
    return false;
  }

  if (yaw_slip_detect_){
    if (body_utime < utime_disable_until_){
      std::cout << "yaw lock disabled until " << utime_disable_until_ << " (" << ((double) (utime_disable_until_ - body_utime)*1E-6) << " secs more)\n";
      return false;
    }
  }


  // Solve FK for feet:
  std::map<std::string, double> jointpos_in;
  std::map<std::string, KDL::Frame > cartpos_out;
  for (size_t i=0; i<  joint_name_.size(); i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(joint_name_[i], joint_position_[i]));
  bool kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,true); // true = flatten tree to absolute transforms
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
    exit(-1);
  }
  Eigen::Isometry3d body_to_l_foot = KDLToEigen(cartpos_out.find( left_standing_link_ )->second);
  Eigen::Isometry3d body_to_r_foot = KDLToEigen(cartpos_out.find( right_standing_link_ )->second);



  Eigen::Isometry3d l_foot_to_r_foot = body_to_l_foot.inverse() *body_to_r_foot ;

  // If not initialized then do so:
  if ( !lock_init_ ){
    world_to_l_foot_original_ =  world_to_body * body_to_l_foot;
    world_to_r_foot_original_ =  world_to_body * body_to_r_foot;
    l_foot_to_r_foot_original_ = l_foot_to_r_foot;

    std::cout << "==============================\n\n\n";
    std::string string0 = print_Isometry3d(world_to_body);
    std::cout << string0 << " captured world body\n";
    std::string string1 = print_Isometry3d(world_to_l_foot_original_);
    std::cout << string1 << " captured world lfoot\n";
    std::string string2 = print_Isometry3d(world_to_r_foot_original_);
    std::cout << string2 << " captured world rfoot\n";

    lock_init_ = true;
    return false;
  }


  std::cout << "\n";
  if (yaw_slip_detect_){
    double l_foot_to_r_foot_rpy[3];
    quat_to_euler( Eigen::Quaterniond(l_foot_to_r_foot.rotation()), l_foot_to_r_foot_rpy[0], l_foot_to_r_foot_rpy[1], l_foot_to_r_foot_rpy[2]);
    double l_foot_to_r_foot_original_rpy[3];
    quat_to_euler( Eigen::Quaterniond(l_foot_to_r_foot_original_.rotation()), l_foot_to_r_foot_original_rpy[0], l_foot_to_r_foot_original_rpy[1], l_foot_to_r_foot_original_rpy[2]);
    double yaw_diff_change = fabs( l_foot_to_r_foot_rpy[2] - l_foot_to_r_foot_original_rpy[2] );
    std::cout <<  "left-right yaw angle: " << l_foot_to_r_foot_original_rpy[2]*180/M_PI << " original | " << l_foot_to_r_foot_rpy[2]*180/M_PI << " now | " << yaw_diff_change*180/M_PI << " change (deg)\n";

    // If a yaw change of more than XX degrees is detected in the kinematics, don't do yaw lock
    if (yaw_diff_change*180/M_PI >  yaw_slip_threshold_degrees_ ){
      utime_disable_until_ = body_utime + yaw_slip_disable_period_*1E6;
      std::stringstream message;
      message << "yaw slippage of "<< (yaw_diff_change*180/M_PI) << " degrees detected. Resetting and disabling the yaw lock until " << utime_disable_until_;
      std::cout << message.str() << "\n";

      bot_core::utime_t warning_message;
      warning_message.utime = body_utime;
      lcm_pub->publish(("YAW_SLIP_DETECTED"), &warning_message);
      bot_core::system_status_t stat_msg;
      stat_msg.utime = 0;
      stat_msg.system = stat_msg.MOTION_ESTIMATION;
      stat_msg.importance = stat_msg.VERY_IMPORTANT;
      stat_msg.frequency = stat_msg.LOW_FREQUENCY;
      stat_msg.value = message.str();
      lcm_pub->publish(("SYSTEM_STATUS"), &stat_msg);

      lock_init_ = false;
      return false;
    }
  }


  // Calculated the mean of orientations inferred by the feet:
  Eigen::Isometry3d world_to_body_using_left = world_to_l_foot_original_ * body_to_l_foot.inverse();
  Eigen::Isometry3d world_to_body_using_right = world_to_r_foot_original_ * body_to_r_foot.inverse();
  // Note: the two above frames could be used to infer footslippage
  Eigen::Quaterniond world_to_body_using_left_quat(world_to_body_using_left.rotation());
  Eigen::Quaterniond world_to_body_using_right_quat(world_to_body_using_right.rotation());
  world_to_body_quat_correction = world_to_body_using_left_quat.slerp(0.5,world_to_body_using_right_quat);


//   std::cout << "==============================\n\n\n";
//   std::string l_string = print_Isometry3d(body_to_l_foot);
//   std::cout << l_string << " lfoot\n";
//   std::string r_string = print_Isometry3d(body_to_r_foot);
//   std::cout << r_string << " rfoot\n";
//
//   std::string l_string2 = print_Isometry3d(world_to_body_using_left);
//   std::cout << l_string2 << " world lfoot\n";
//   std::string r_string2 = print_Isometry3d(world_to_body_using_right);
//   std::cout << r_string2 << " world rfoot\n";
//   std::cout << world_to_body_quat_update.w() << ", " << world_to_body_quat_update.w() << ", " << world_to_body_quat_update.w()
//             << ", " << world_to_body_quat_update.w() <<" output\n";


  double rpy_update[3];
  quat_to_euler( Eigen::Quaterniond(world_to_body_quat_correction.w(), world_to_body_quat_correction.x(),
                                    world_to_body_quat_correction.y(), world_to_body_quat_correction.z()) ,
                 rpy_update[0], rpy_update[1], rpy_update[2]);
  std::cout << rpy_update[0] << ", " << rpy_update[1] << " " << rpy_update[2] << " output rpy [rad]\n";
  std::cout << rpy_update[0]*180/M_PI << ", " << rpy_update[1]*180/M_PI << " " << rpy_update[2]*180/M_PI << " output rpy [deg]\n";
  std::cout << "\n";

  // testing hack - remove me:
  //rpy_update[2] =+ 1;
  //world_to_body_quat_correction = euler_to_quat(rpy_update[0], rpy_update[1], rpy_update[2]);


  
  return true;
}
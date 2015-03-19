// if transitioned to standing, capture yaw
// there after, issue this yaw as a correction to 
// to the state estimate

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>


#include <ConciseArgs>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/pronto/atlas_state_t.hpp>
#include <lcmtypes/pronto/system_status_t.hpp>
#include <lcmtypes/pronto/utime_t.hpp>
#include <pronto_utils/pronto_math.hpp>
#include <pronto_utils/pronto_joint_tools.hpp>

using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope
using namespace boost;
using namespace Eigen;


struct CommandLineConfig
{
    std::string output_channel;
    int correction_period;

    // should you detect a yaw slip of yaw_slip_threshold_degrees, dont update lock for yaw_slip_disable_period seconds
    bool yaw_slip_detect;
    double yaw_slip_threshold_degrees;
    double yaw_slip_disable_period;
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    std::vector<std::string> joint_names_;
    std::vector<float>   joint_positions_;

    // Poses of the l and r feet when robot first became standing or manipulating:
    Eigen::Isometry3d world_to_l_foot_original_, world_to_r_foot_original_;
    bool joint_angles_init_; // have received some joint angles
    bool lock_init_; // is the yaw lock active

    // Used in foot slip detection:
    Eigen::Isometry3d l_foot_to_r_foot_original_;
    int64_t utime_disable_until_;

    void atlasStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::atlas_state_t* msg);
    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
    const CommandLineConfig cl_cfg_;
    int64_t counter_;

    int last_controller_state_;
    double current_yaw_;
    int correction_state_; // state in which we can apply the change
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  JointUtils* joint_utils = new JointUtils();
  joint_names_ = joint_utils->atlas_joint_names;
  joint_angles_init_ = false;
  lock_init_ = false;
  utime_disable_until_ = 0; // don't disable at start (but will be uninitialized)

  lcm_->subscribe( "ATLAS_STATE" ,&App::atlasStateHandler,this);
  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);
  lcm_->subscribe( "CONTROLLER_STATUS" ,&App::controllerStatusHandler,this);

  counter_=0;

  last_controller_state_ = pronto::controller_status_t::UNKNOWN; // 
  correction_state_ = pronto::controller_status_t::STANDING;
  //correction_state_ = pronto::controller_status_t::DUMMY; // for testing

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

void App::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  last_controller_state_ = msg->state;
}

void App::atlasStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::atlas_state_t* msg){
  joint_angles_init_ = true;
  joint_positions_ = msg->joint_position;
}


void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  if (!joint_angles_init_){
    std::cout << "no joints received yet, returning\n";
    return;
  }

  // Occasionally send a correction when in either standing or manipulating mode
  if (counter_ % cl_cfg_.correction_period != 0){ // Correct the yaw drift every X tics
    counter_++;
    return;
  }else{
    counter_++;
  }
  if (!( (last_controller_state_ == pronto::controller_status_t::STANDING) ||
       (last_controller_state_ == pronto::controller_status_t::MANIPULATING) )){
    std::cout << msg->utime << " not in standing or manipulation mode, not correcting\n";
    lock_init_ = false;
    return;
  }

  if (cl_cfg_.yaw_slip_detect){
    if (msg->utime < utime_disable_until_){
      std::cout << "yaw lock disabled until " << utime_disable_until_ << " (" << ((double) (utime_disable_until_ - msg->utime)*1E-6) << " secs more)\n";
      return;
    }
  }


  // Solve FK for feet:
  map<string, double> jointpos_in;
  map<string, KDL::Frame > cartpos_out;
  for (size_t i=0; i<  joint_names_.size(); i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(joint_names_[i], joint_positions_[i]));
  bool kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,true); // true = flatten tree to absolute transforms
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    exit(-1);
  }
  Eigen::Isometry3d body_to_l_foot = KDLToEigen(cartpos_out.find("l_foot")->second);
  Eigen::Isometry3d body_to_r_foot = KDLToEigen(cartpos_out.find("r_foot")->second);

  // Get the Body Position:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1],
                                               msg->orientation[2], msg->orientation[3]);
  world_to_body.rotate(quat);


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
    return;
  }


  std::cout << "\n";
  if (cl_cfg_.yaw_slip_detect){
    double l_foot_to_r_foot_rpy[3];
    quat_to_euler( Eigen::Quaterniond(l_foot_to_r_foot.rotation()), l_foot_to_r_foot_rpy[0], l_foot_to_r_foot_rpy[1], l_foot_to_r_foot_rpy[2]);
    double l_foot_to_r_foot_original_rpy[3];
    quat_to_euler( Eigen::Quaterniond(l_foot_to_r_foot_original_.rotation()), l_foot_to_r_foot_original_rpy[0], l_foot_to_r_foot_original_rpy[1], l_foot_to_r_foot_original_rpy[2]);
    double yaw_diff_change = fabs( l_foot_to_r_foot_rpy[2] - l_foot_to_r_foot_original_rpy[2] );
    std::cout <<  "left-right yaw angle: " << l_foot_to_r_foot_original_rpy[2]*180/M_PI << " original | " << l_foot_to_r_foot_rpy[2]*180/M_PI << " now | " << yaw_diff_change*180/M_PI << " change (deg)\n";

    // If a yaw change of more than XX degrees is detected in the kinematics, don't do yaw lock
    if (yaw_diff_change*180/M_PI >  cl_cfg_.yaw_slip_threshold_degrees ){
      utime_disable_until_ = msg->utime+ cl_cfg_.yaw_slip_disable_period*1E6;
      std::stringstream message;
      message << "yaw slippage of "<< (yaw_diff_change*180/M_PI) << " degrees detected. Resetting and disabling the yaw lock until " << utime_disable_until_;
      std::cout << message.str() << "\n";

      pronto::utime_t warning_message;
      warning_message.utime = msg->utime;
      lcm_->publish(("YAW_SLIP_DETECTED"), &warning_message);
      pronto::system_status_t stat_msg;
      stat_msg.utime = 0;
      stat_msg.system = stat_msg.MOTION_ESTIMATION;
      stat_msg.importance = stat_msg.VERY_IMPORTANT;
      stat_msg.frequency = stat_msg.LOW_FREQUENCY;
      stat_msg.value = message.str();
      lcm_->publish(("SYSTEM_STATUS"), &stat_msg);

      lock_init_ = false;
      return;
    }
  }


  // Calculated the mean of orientations inferred by the feet:
  Eigen::Isometry3d world_to_body_using_left = world_to_l_foot_original_ * body_to_l_foot.inverse();
  Eigen::Isometry3d world_to_body_using_right = world_to_r_foot_original_ * body_to_r_foot.inverse();
  // Note: the two above frames could be used to infer footslippage
  Eigen::Quaterniond world_to_body_using_left_quat(world_to_body_using_left.rotation());
  Eigen::Quaterniond world_to_body_using_right_quat(world_to_body_using_right.rotation());
  Eigen::Quaterniond world_to_body_quat_update = world_to_body_using_left_quat.slerp(0.5,world_to_body_using_right_quat);

  std::cout << msg->utime << ": sending POSE_YAW_LOCK\n";
  bot_core::pose_t out;
  out.utime = msg->utime;
  out.pos[0] = 0;
  out.pos[1] = 0;
  out.pos[2] = 0;
  out.orientation[0] = world_to_body_quat_update.w();
  out.orientation[1] = world_to_body_quat_update.x();
  out.orientation[2] = world_to_body_quat_update.y();
  out.orientation[3] = world_to_body_quat_update.z();
  lcm_->publish(cl_cfg_.output_channel , &out);


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
  quat_to_euler( Eigen::Quaterniond(world_to_body_quat_update.w(), world_to_body_quat_update.x(),
                                    world_to_body_quat_update.y(), world_to_body_quat_update.z()) ,
                 rpy_update[0], rpy_update[1], rpy_update[2]);
  std::cout << rpy_update[0] << ", " << rpy_update[1] << " " << rpy_update[2] << " output rpy [rad]\n";
  std::cout << rpy_update[0]*180/M_PI << ", " << rpy_update[1]*180/M_PI << " " << rpy_update[2]*180/M_PI << " output rpy [deg]\n";
  std::cout << "\n";


}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.output_channel = "POSE_YAW_LOCK";
  cl_cfg.correction_period = 333; // 333 is one sec of POSE_BODY

  // Added to detect yae slip:
  cl_cfg.yaw_slip_detect = false;
  cl_cfg.yaw_slip_threshold_degrees = 1.5; // degrees
  cl_cfg.yaw_slip_disable_period = 5; // seconds

  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.output_channel, "o", "output_channel","Output measurement channel");
  opt.add(cl_cfg.correction_period, "p", "correction_period","Period (in samples) between corrections");
  opt.add(cl_cfg.yaw_slip_detect, "yd", "yaw_slip_detect","Try to detect foot slippage");
  opt.add(cl_cfg.yaw_slip_threshold_degrees, "ya", "yaw_slip_threshold_degrees","Threshold in yaw we detect");
  opt.add(cl_cfg.yaw_slip_disable_period, "yp", "yaw_slip_disable_period","Amount of time to disable lock after we have detected (sec)");
  opt.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App app(lcm, cl_cfg);
  cout << "Yaw Lock Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());

  return 0;
}

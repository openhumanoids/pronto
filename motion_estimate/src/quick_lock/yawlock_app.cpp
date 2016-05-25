#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <model-client/model-client.hpp>
#include <ConciseArgs>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/pronto/behavior_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/bot_core/system_status_t.hpp>
#include <lcmtypes/bot_core/utime_t.hpp>
#include <pronto_utils/pronto_math.hpp>

#include <mav_state_est/mav-est-yawlock/yawlock.hpp>


struct CommandLineConfig
{
    std::string output_channel;
    int correction_period;

    // should you detect a yaw slip of yaw_slip_threshold_degrees, dont update lock for yaw_slip_disable_period seconds
    bool yaw_slip_detect;
    double yaw_slip_threshold_degrees;
    double yaw_slip_disable_period;

    std::string behavior_input_mode;

    std::string left_standing_link;
    std::string right_standing_link;
};


class App{
  public:
    App(lcm::LCM* &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

  private:
    lcm::LCM* lcm_;
    const CommandLineConfig cl_cfg_;
    boost::shared_ptr<ModelClient> model_;
    YawLock* yaw_lock_;

    Eigen::Isometry3d world_to_body_;
    bool world_to_body_init_;

    void jointStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
    void robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg);    

};

App::App(lcm::LCM* &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));

  yaw_lock_ = new YawLock(lcm_, &(*lcm_), model_);
  yaw_lock_->setParameters(cl_cfg_.correction_period, cl_cfg_.yaw_slip_detect, 
    cl_cfg_.yaw_slip_threshold_degrees, cl_cfg_.yaw_slip_disable_period );

  lcm_->subscribe( "CORE_ROBOT_STATE" ,&App::jointStateHandler,this);
  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);

  yaw_lock_->setStandingLinks( cl_cfg_.left_standing_link, cl_cfg_.right_standing_link );

  if (cl_cfg_.behavior_input_mode == "CONTROLLER_STATUS"){
    // MIT controller:
    lcm_->subscribe( "CONTROLLER_STATUS" ,&App::controllerStatusHandler,this);
  }else if (cl_cfg_.behavior_input_mode == "ROBOT_BEHAVIOR") {
    // ROBOT_BEHAVIOR comes from IHMC or BDI API
    lcm_->subscribe( "ROBOT_BEHAVIOR" ,&App::robotBehaviorHandler,this);
  }else{
    std::cout << "behavior_input_mode not recognised: CONTROLLER_STATUS or ROBOT_BEHAVIOR\n";
    exit(-1);
  }

  world_to_body_init_ = false;

}

void App::robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->behavior == pronto::behavior_t::BEHAVIOR_STAND){
    is_robot_standing = true; 
  }else if (msg->behavior == pronto::behavior_t::BEHAVIOR_MANIPULATE){
    is_robot_standing = true; 
  }

  yaw_lock_->setIsRobotStanding(is_robot_standing);
}


void App::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  bool is_robot_standing = false;

  // If standing or manipulating:
  if (msg->state == pronto::controller_status_t::STANDING){
    is_robot_standing = true; 
  }else if (msg->state == pronto::controller_status_t::MANIPULATING){
    is_robot_standing = true; 
  }

  yaw_lock_->setIsRobotStanding(is_robot_standing);
}

void App::jointStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){

  if ( !world_to_body_init_ ){
    std::cout << "YawLock: pose body now received yet, returning\n";
    return;
  }

  yaw_lock_->setJointState(msg->joint_position, msg->joint_name);

  Eigen::Quaterniond world_to_body_quat_correction;  
  if (yaw_lock_->getCorrection(world_to_body_, msg->utime, world_to_body_quat_correction) ){

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

}


void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  world_to_body_.setIdentity();
  world_to_body_.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1],
                                               msg->orientation[2], msg->orientation[3]);
  world_to_body_.rotate(quat);
  world_to_body_init_ = true;
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.output_channel = "POSE_YAW_LOCK";
  cl_cfg.behavior_input_mode = "CONTROLLER_STATUS";
  cl_cfg.correction_period = 333; // 333 is one sec of POSE_BODY

  cl_cfg.left_standing_link = "l_foot";
  cl_cfg.right_standing_link = "r_foot";

  // Added to detect yaw slip:
  cl_cfg.yaw_slip_detect = false;
  cl_cfg.yaw_slip_threshold_degrees = 1.5; // degrees
  cl_cfg.yaw_slip_disable_period = 5; // seconds

  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.output_channel, "o", "output_channel","Output measurement channel");
  opt.add(cl_cfg.behavior_input_mode, "b", "behavior","Should we listen to CONTROLLER_STATUS or ROBOT_BEHAVIOR for robot behavior");  
  opt.add(cl_cfg.correction_period, "p", "correction_period","Period (in samples) between corrections");
  opt.add(cl_cfg.yaw_slip_detect, "yd", "yaw_slip_detect","Try to detect foot slippage");
  opt.add(cl_cfg.yaw_slip_threshold_degrees, "ya", "yaw_slip_threshold_degrees","Threshold in yaw we detect");
  opt.add(cl_cfg.yaw_slip_disable_period, "yp", "yaw_slip_disable_period","Amount of time to disable lock after we have detected (sec)");
  opt.add(cl_cfg.left_standing_link, "lf", "left_standing_link","left_standing_link");
  opt.add(cl_cfg.right_standing_link, "rf", "right_standing_link","right_standing_link");
  opt.parse();

  lcm::LCM* lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App app(lcm, cl_cfg);
  std::cout << "Yaw Lock Tool ready" << std::endl << "============================" << std::endl;
  while(0 == lcm->handle());

  return 0;
}

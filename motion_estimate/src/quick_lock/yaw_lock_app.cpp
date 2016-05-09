#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <model-client/model-client.hpp>
#include <ConciseArgs>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/bot_core/system_status_t.hpp>
#include <lcmtypes/bot_core/utime_t.hpp>
#include <pronto_utils/pronto_math.hpp>

#include <yaw_lock/yaw_lock.hpp>


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
    const CommandLineConfig cl_cfg_;
    boost::shared_ptr<ModelClient> model_;
    YawLock* yaw_lock_;

    void jointStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);

};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));

  yaw_lock_ = new YawLock(lcm_, model_);
  yaw_lock_->setParameters(cl_cfg_.correction_period, cl_cfg_.yaw_slip_detect, 
    cl_cfg_.yaw_slip_threshold_degrees, cl_cfg_.yaw_slip_disable_period );

  lcm_->subscribe( "CORE_ROBOT_STATE" ,&App::jointStateHandler,this);
  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);
  lcm_->subscribe( "CONTROLLER_STATUS" ,&App::controllerStatusHandler,this);

}

void App::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  yaw_lock_->setControllerState(msg->state);  
}

void App::jointStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  yaw_lock_->setJointState(msg->joint_position, msg->joint_name);
}


void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  if ( !yaw_lock_->getJointAnglesInit() ){
    std::cout << "no joints received yet, returning\n";
    return;
  }

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

}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.output_channel = "POSE_YAW_LOCK";
  cl_cfg.correction_period = 333; // 333 is one sec of POSE_BODY

  // Added to detect yaw slip:
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
  std::cout << "Yaw Lock Tool ready" << std::endl << "============================" << std::endl;
  while(0 == lcm->handle());

  return 0;
}

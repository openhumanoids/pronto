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

#include <ConciseArgs>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <pronto_utils/pronto_math.hpp>

using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope
using namespace boost;
using namespace Eigen;


struct CommandLineConfig
{
    std::string output_channel;
    double initial_yaw_angle;
    int correction_period;
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
    const CommandLineConfig cl_cfg_;
    int64_t counter_;

    int last_controller_state_;
    double reference_yaw_angle_;
    double current_yaw_;
    int correction_state_; // state in which we can apply the change
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);
  lcm_->subscribe( "CONTROLLER_STATUS" ,&App::controllerStatusHandler,this);

  counter_=0;

  last_controller_state_ = pronto::controller_status_t::UNKNOWN; // 
  correction_state_ = pronto::controller_status_t::STANDING;
  //correction_state_ = pronto::controller_status_t::DUMMY; // for testing

  reference_yaw_angle_ = cl_cfg_.initial_yaw_angle;
}


void App::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  if (last_controller_state_ != msg->state){ 
    if(msg->state == correction_state_){ // if state has changed and is now standing, then update the ref angle
      reference_yaw_angle_ =current_yaw_;
      std::cout << msg->utime << ": " << reference_yaw_angle_*180.0/M_PI << "degrees. Changing to standing, updating reference yaw lock\n";
    }
  }

  last_controller_state_ = msg->state;
}

void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  // Always cache the current yaw angle:
  double temp_r, temp_p;
  quat_to_euler( Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3])  , temp_r, temp_p, current_yaw_);

  // If haven't recently sent a correction and in the correction state, then send one:
  counter_++;
  if (counter_ % 10000 == 0){
    std::cout << msg->utime << ": POSE_YAW_LOCK active\n";
  }
  if (counter_ % cl_cfg_.correction_period != 0){ // Correct the yaw drift every X tics	
    return;
  }
  if(last_controller_state_ != correction_state_){
    std::cout << msg->utime << " not in standing mode, not correcting\n";
    return;
  }

  std::cout << msg->utime << ": " << reference_yaw_angle_*180.0/M_PI << " degrees. sending POSE_YAW_LOCK\n";
  bot_core::pose_t out;
  out.utime = msg->utime;
  out.pos[0] = 0;
  out.pos[1] = 0;
  out.pos[2] = 0;
  Eigen::Quaterniond quat =euler_to_quat(0,0, reference_yaw_angle_ );
  out.orientation[0] = quat.w();
  out.orientation[1] = quat.x();
  out.orientation[2] = quat.y();
  out.orientation[3] = quat.z();
  lcm_->publish(cl_cfg_.output_channel , &out);
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.output_channel = "POSE_YAW_LOCK";
  cl_cfg.initial_yaw_angle = 0.0;
  cl_cfg.correction_period = 333; // 333 is one sec of POSE_BODY
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.output_channel, "o", "output_channel","Output measurement channel");
  opt.add(cl_cfg.initial_yaw_angle, "y", "initial_yaw_angle","Yaw angle in rads");
  opt.add(cl_cfg.correction_period, "p", "correction_period","Period (in samples) between corrections");
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

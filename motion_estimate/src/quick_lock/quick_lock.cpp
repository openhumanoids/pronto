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
#include <lcmtypes/pronto/double_array_t.hpp>
#include <lcmtypes/pronto/indexed_measurement_t.hpp>
#include <lcmtypes/pronto/filter_state_t.hpp>

#include <pronto_utils/pronto_math.hpp>

using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope
using namespace boost;
using namespace Eigen;


struct CommandLineConfig
{
    int mode;
    std::string output_channel;
    float xyz_cov;
    float yaw_cov;
    bool correct_all_modes;
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void filterStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::filter_state_t* msg);
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);

    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseLaserHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    const CommandLineConfig cl_cfg_;

    int last_controller_state_;
    int64_t counter_;

    bot_core::pose_t laser_pose_msg_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){

  lcm_->subscribe( "STATE_ESTIMATOR_STATE_LASER" ,&App::filterStateHandler,this);
  lcm_->subscribe( "CONTROLLER_STATUS" ,&App::controllerStatusHandler,this);

  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);
  lcm_->subscribe( "POSE_BODY_LASER" ,&App::poseLaserHandler,this);

  last_controller_state_ = -1; // none receieved
  counter_=0;
}


void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
}


void App::poseLaserHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  laser_pose_msg_ = *msg;
}


void App::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg){
  if (last_controller_state_ != msg->state){
    if(msg->state == 1){
      std::cout << "changing to standing, disable lock\n";
    }else if (msg->state == 2){
      std::cout << "changing to walking, enable lock\n";
    }
  }

  last_controller_state_ = msg->state;
}


void App::filterStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::filter_state_t* msg){
  if (last_controller_state_ == -1){
    std::cout << "cannot init - no controller state\n";
    return;
  }

  if (!cl_cfg_.correct_all_modes){
    if (last_controller_state_!=2){
//      std::cout << "not walking, lock disabled, returning\n";
//      return;
    }
  }

  counter_++;
  if (counter_ % 10000 == 0){
    std::cout << msg->utime << " STATE_ESTIMATOR_STATE_LASER\n";
  }

  //the indices used to correct SE:
  //8: yaw?
  //9,10,11: xyz

  pronto::indexed_measurement_t out;
  out.utime = msg->utime;
  out.state_utime = msg->utime;


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


/*
  out.measured_dim = 6;
  out.z_indices = {6,7,8,9,10,11}; // yaw, x, y, z
  out.z_effective = {msg->state[6], msg->state[7], msg->state[8], msg->state[9], msg->state[10], msg->state[11]};
  out.measured_cov_dim = 36;
  out.R_effective = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0 };
  out.R_effective[0]  = cl_cfg_.yaw_cov; // chi0
  out.R_effective[7]  = cl_cfg_.yaw_cov; // chi1
  out.R_effective[14]  = cl_cfg_.yaw_cov;// chi2
  out.R_effective[21]  = cl_cfg_.xyz_cov;// x
  out.R_effective[28] = cl_cfg_.xyz_cov; // y
  out.R_effective[35] = cl_cfg_.xyz_cov; // z
*/



  lcm_->publish(cl_cfg_.output_channel , &out);
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.mode = 0;
  cl_cfg.output_channel = "GPF_MEASUREMENT_QUICK_LOCK";
  cl_cfg.correct_all_modes = false;
  // higher is weaker.
  // 100 seemed necessary to clamp close to current position
  //cl_cfg.xyz_cov = 200; // looked vaguely appropriate
  //cl_cfg.yaw_cov = pow(5.0*M_PI/180.0,2); // this is taken from viewer UI
  // 200/400 were used in the experiments for the walking journal paper and seem about right
  cl_cfg.xyz_cov = 200;
  cl_cfg.yaw_cov = 400;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.mode, "f", "mode","Filter Type");
  opt.add(cl_cfg.output_channel, "o", "output_channel","Output measurement channel");
  opt.add(cl_cfg.correct_all_modes, "a", "correct_all_modes","Correct during all modes");
  opt.add(cl_cfg.xyz_cov, "x", "xyz","Linear Covariance Rate");
  opt.add(cl_cfg.yaw_cov, "y", "yaw","Yaw Covariance Rate");
  opt.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App app(lcm, cl_cfg);
  cout << "Tool ready1" << endl << "============================" << endl;
  while(0 == lcm->handle());

  return 0;
}

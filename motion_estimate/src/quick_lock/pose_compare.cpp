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
#include <lcmtypes/pronto/double_array_t.hpp>
#include <lcmtypes/mav/indexed_measurement_t.hpp>
#include <lcmtypes/mav/filter_state_t.hpp>

#include <pronto_utils/pronto_math.hpp>

using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope
using namespace boost;
using namespace Eigen;


struct CommandLineConfig
{
    int mode;
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseLaserHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    const CommandLineConfig cl_cfg_;

    bot_core::pose_t laser_pose_msg_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){

  lcm_->subscribe( "POSE_BODY" ,&App::poseHandler,this);
  lcm_->subscribe( "POSE_BODY_LASER" ,&App::poseLaserHandler,this);

}


void App::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  double rpy[3], rpyl[3];
  quat_to_euler( Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3]) , rpy[0],rpy[1],rpy[2]);
  quat_to_euler( Eigen::Quaterniond(laser_pose_msg_.orientation[0], laser_pose_msg_.orientation[1], laser_pose_msg_.orientation[2], laser_pose_msg_.orientation[3]) , rpyl[0],rpyl[1],rpyl[2]);

  double dposroll  = (-rpy[0] + rpyl[0])*180.0/M_PI ;
  double dpospitch = (-rpy[1] + rpyl[1])*180.0/M_PI ;
  double dposyaw   = (-rpy[2] + rpyl[2])*180.0/M_PI ;
  double dposx = -msg->pos[0] + laser_pose_msg_.pos[0];
  double dposy = -msg->pos[1] + laser_pose_msg_.pos[1];
  double dposz = -msg->pos[2] + laser_pose_msg_.pos[2];
  //std::cout << dposx << "\n";

  pronto::double_array_t out;
  out.utime = msg->utime;
  out.num_values = 6;
  out.values = {dposroll, dpospitch, dposyaw, dposx, dposy, dposz};
  lcm_->publish("LOCK_ERROR" , &out);

}


void App::poseLaserHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  laser_pose_msg_ = *msg;
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.mode = 0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.mode, "f", "mode","Filter Type");
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

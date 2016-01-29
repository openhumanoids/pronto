#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <lcmtypes/pronto/robot_state_t.hpp>
#include <lcmtypes/pronto/joint_state_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>

using namespace std;
///////////////////////////////////////////////////////////////
class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, double offset_);
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    double offset_;
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::joint_state_t* msg);

};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, double offset_):lcm_(lcm_), offset_( offset_){
  lcm_->subscribe("MULTISENSE_STATE",&App::multisenseHandler,this);
}



void App::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                              const  pronto::joint_state_t* msg){
  vector<string>::iterator it;
  vector<string> vec;
  vec = msg->joint_name;
  it=find(vec.begin(),vec.end(),"hokuyo_joint");
  if(it != vec.end()){
    int hokuyo_idx = (it-vec.begin());

    double angle = msg->joint_position[hokuyo_idx] +  offset_;
    // publish message for bot_frames
    bot_core::rigid_transform_t preToPostFrame;
    preToPostFrame.utime = msg->utime;
    preToPostFrame.trans[0] = 0;
    preToPostFrame.trans[1] = 0;
    preToPostFrame.trans[2] = 0;
    preToPostFrame.quat[0] = std::cos(angle/2);
    preToPostFrame.quat[1] = 0;
    preToPostFrame.quat[2] = 0;
    preToPostFrame.quat[3] = std::sin(angle/2);
    lcm_->publish("PRE_SPINDLE_TO_POST_SPINDLE",
                               &preToPostFrame);
  }


/*
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = msg->utime;
  // Pelvis Pose:
  robot_state_msg.pose.translation.x =0;
  robot_state_msg.pose.translation.y =0;
  robot_state_msg.pose.translation.z =0;
  robot_state_msg.pose.rotation.w = 1;
  robot_state_msg.pose.rotation.x = 0;
  robot_state_msg.pose.rotation.y = 0;
  robot_state_msg.pose.rotation.z = 0;
  robot_state_msg.twist.linear_velocity.x  = 0;
  robot_state_msg.twist.linear_velocity.y  = 0;
  robot_state_msg.twist.linear_velocity.z  = 0;
  robot_state_msg.twist.angular_velocity.x = 0;
  robot_state_msg.twist.angular_velocity.y = 0;
  robot_state_msg.twist.angular_velocity.z = 0;  
  
  for (size_t i = 0; i < msg->joint_position.size(); i++)  {
//    robot_state_msg.joint_name.push_back( atlas_joints_.name[i] );
    robot_state_msg.joint_position.push_back( msg->joint_position[i] );
    robot_state_msg.joint_velocity.push_back( num_velocity[i]);
    robot_state_msg.joint_effort.push_back( num_velocity[i] );
  }  
  robot_state_msg.joint_name = joint_utils_.atlas_joint_names;  

  robot_state_msg.num_joints = robot_state_msg.joint_position.size();
  
  lcm_->publish("EST_ROBOT_STATE_ECHO", &robot_state_msg);  
*/
}


int main(int argc, char ** argv) {
  double offset = 0.0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(offset, "o", "offset","Joint Offset");
  opt.parse();
  

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm,offset);
  std::cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}

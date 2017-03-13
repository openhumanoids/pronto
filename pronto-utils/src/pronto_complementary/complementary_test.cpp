// Basic demo application for 

#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pronto_complementary/complementary.hpp>
#include <ConciseArgs>

#include <pronto_utils/pronto_math.hpp>

using namespace std;

struct CommandLineConfig
{
  std::string input_channel;
  double dt;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }

  private:
    const CommandLineConfig cl_cfg_;    
    
    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    Complementary* comp_;

    // IMU
    void microstrainHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg);

};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_)
{
  comp_ = new Complementary(cl_cfg_.dt); // dt = 0.01 sec
  lcm_recv_->subscribe( cl_cfg_.input_channel, &App::microstrainHandler,this);
  cout <<"Complementary Test Constructed\n";
}


void _quat_rotate_to (const double rot[4], const double v[3], double r[3])
{
    double ab  =  rot[0]*rot[1], ac = rot[0]*rot[2], ad  =  rot[0]*rot[3];
    double nbb = -rot[1]*rot[1], bc = rot[1]*rot[2], bd  =  rot[1]*rot[3];
    double ncc = -rot[2]*rot[2], cd = rot[2]*rot[3], ndd = -rot[3]*rot[3];

    r[0] = 2*((ncc + ndd)*v[0] + (bc - ad)*v[1] + (ac + bd)*v[2]) + v[0];
    r[1] = 2*((ad + bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2]) + v[1];
    r[2] = 2*((bd - ac)*v[0] + (ab + cd)*v[1] + (nbb + ncc)*v[2]) + v[2];
}


void App::microstrainHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::ins_t* msg){

  // 1a take raw imu measurements, transform into: x forward +, y left +, z up +
  // Apply unit rotation (only needed because microstrain orientation is z down
  //Eigen::Quaterniond rot_quat_Eigen = euler_to_quat(M_PI, 0, 0);
  Eigen::Quaterniond rot_quat_Eigen = euler_to_quat(0, M_PI, 0);
//  Eigen::Quaterniond rot_quat_Eigen = euler_to_quat(0, M_PI, 0);

  double rot_quat[4];
  rot_quat[0] = rot_quat_Eigen.w();
  rot_quat[1] = rot_quat_Eigen.x();
  rot_quat[2] = rot_quat_Eigen.y();
  rot_quat[3] = rot_quat_Eigen.z();

  double body_accel[3];
  double body_gyro[3];
  _quat_rotate_to(rot_quat, msg->accel, body_accel);
  _quat_rotate_to(rot_quat, msg->gyro, body_gyro);

  Eigen::Map<Eigen::Vector3d> accData(body_accel);
  Eigen::Map<Eigen::Vector3d> gyrData(body_gyro);

  // 1b Otherwise don't
  //Eigen::Vector3d accData = Eigen::Vector3d(msg->accel);
  //Eigen::Vector3d gyrData = Eigen::Vector3d(msg->gyro);

  // 2 Update filter
  comp_->update(accData, gyrData);
  double pitch = comp_->getPitch();
  double roll = comp_->getRoll();
  double yaw = comp_->getYaw();
  Eigen::Quaterniond motion_R = euler_to_quat(roll, pitch, yaw);

  // 3 output, including misc outputs for debug
  bot_core::pose_t legodo_msg;
  legodo_msg.utime = msg->utime;
  legodo_msg.pos[0] = 0;
  legodo_msg.pos[1] = 0;
  legodo_msg.pos[2] = 0;
  legodo_msg.orientation[0] = motion_R.w();
  legodo_msg.orientation[1] = motion_R.x();
  legodo_msg.orientation[2] = motion_R.y();
  legodo_msg.orientation[3] = motion_R.z();

  legodo_msg.vel[0] = roll*180/M_PI; // just for debug
  legodo_msg.vel[1] = pitch*180/M_PI;
  legodo_msg.vel[2] = yaw;

  legodo_msg.accel[0] = comp_->rollAcc*180/M_PI; // just for debug
  legodo_msg.accel[1] = comp_->pitchAcc*180/M_PI;
  legodo_msg.accel[2] = 0;

  lcm_pub_->publish("POSE_BODY_ALT", &legodo_msg);
}


int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.input_channel = "MICROSTRAIN_INS";
  cl_cfg.dt = 0.01;

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel");
  parser.add(cl_cfg.dt, "d", "dt", "dt - typical elapsed time between measurments");
  parser.parse();
  cout << cl_cfg.input_channel << " is input_channel\n";
  cout << cl_cfg.dt << " is dt\n";
  
  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  lcm_recv = lcm_pub;

  App fo= App(lcm_recv, lcm_pub, cl_cfg);
  while(0 == lcm_recv->handle());
}


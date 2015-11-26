#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include "lidar-odometry.hpp"
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/pronto/pointcloud_t.hpp>
#include <ConciseArgs>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

using namespace std;

struct CommandLineConfig
{
  bool use_velodyne;
  bool init_with_message; // initialize off of a pose or vicon
  std::string output_channel;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;

    BotParam* botparam_;
    BotFrames* botframes_;

    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);
    void pointCloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::pointcloud_t* msg);
    LidarOdom* lidarOdom_;

    Eigen::Isometry3d body_to_lidar_; // Fixed tf from the lidar to the robot's base link
    Eigen::Isometry3d world_to_body_init_; // Captures the position of the body frame in world at launch
    Eigen::Isometry3d world_to_body_now_; // running position estimate

    // Init handlers:
    void rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void initState( const double trans[3], const double quat[4]);
    bool pose_initialized_;

    bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime);

    int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d& mat);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_) : 
       lcm_(lcm_), cl_cfg_(cl_cfg_){
  
  // Set up frames and config:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  if (cl_cfg_.use_velodyne){
    lcm_->subscribe("VELODYNE_HORIZONTAL",&App::pointCloudHandler,this);
    int status = get_trans_with_utime( botframes_ ,  "VELODYNE", "body"  , 0, body_to_lidar_);
  }else{
    lcm_->subscribe("SCAN",&App::lidarHandler,this);
    int status = get_trans_with_utime( botframes_ ,  "SCAN", "body"  , 0, body_to_lidar_);
  }

  pose_initialized_ = false;
  if (!cl_cfg_.init_with_message){
    std::cout << "Init internal est using default\n";
    world_to_body_init_ = Eigen::Isometry3d::Identity();

    /*
    world_to_body_init_.setIdentity();
    world_to_body_init_.translation()  << 1.2, 1.67, 0.32;
    Eigen::Quaterniond q(-0.045668, -0.004891, -0.00909, 0.9989);
    world_to_body_init_.rotate(q);
    bot_core::pose_t pose_msg_body = getPoseAsBotPose( world_to_lidar_init_ , 0);
    lcm_->publish("POSE_BODY_ALT", &pose_msg_body );
    pose_initialized_ = true;
    */

    pose_initialized_ = true;
  }else{
    lcm_->subscribe("VICON_BODY|VICON_FRONTPLATE",&App::rigidTransformInitHandler,this);
    lcm_->subscribe("POSE_VICON",&App::poseInitHandler,this);
    std::cout << "Waiting for Init message to LIDAR estimator\n";
  }

  lidarOdom_ = new LidarOdom(lcm_);
}

int App::get_trans_with_utime(BotFrames *bot_frames,
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

bot_core::pose_t App::getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();
  pose_msg.orientation[1] =  r_x.x();
  pose_msg.orientation[2] =  r_x.y();
  pose_msg.orientation[3] =  r_x.z();
  return pose_msg;
}


void App::lidarHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (!pose_initialized_){
    std::cout << "Estimate not initialised, exiting lidarHandler\n";
    return;
  }

  // 1. Update LIDAR Odometry
  std::vector<float> ranges_copy = msg->ranges;
  float* ranges = &ranges_copy[0];
  lidarOdom_->doOdometry(ranges, msg->nranges, msg->rad0, msg->radstep, msg->utime);

  // 2. Determine the body position using the LIDAR motion estimate:
  Eigen::Isometry3d lidar_init_to_lidar_now = lidarOdom_->getCurrentPose();
  Eigen::Isometry3d world_to_lidar_now = world_to_body_init_*body_to_lidar_*lidar_init_to_lidar_now;
  world_to_body_now_ = world_to_lidar_now * body_to_lidar_.inverse();

  //bot_core::pose_t pose_msg = getPoseAsBotPose( lidar_init_to_lidar_now , msg->utime);
  //lcm_->publish("POSE_BODY_ALT", &pose_msg );
  bot_core::pose_t pose_msg_body = getPoseAsBotPose( world_to_body_now_ , msg->utime);
  lcm_->publish(cl_cfg_.output_channel, &pose_msg_body );
}


void App::pointCloudHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  pronto::pointcloud_t* msg){
  if (!pose_initialized_){
    std::cout << "Estimate not initialised, exiting pointCloudHandler\n";
    return;
  }

  // 1. Update LIDAR Odometry
  std::vector<float> x;
  std::vector<float> y;
  for (int i=0;i<msg->n_points; i++){
    x.push_back( msg->points[i][0] );
    y.push_back( msg->points[i][1] );
  }

  lidarOdom_->doOdometry(x, y, msg->n_points, msg->utime);

  // 2. Determine the body position using the LIDAR motion estimate:
  Eigen::Isometry3d lidar_init_to_lidar_now = lidarOdom_->getCurrentPose();
  Eigen::Isometry3d world_to_lidar_now = world_to_body_init_*body_to_lidar_*lidar_init_to_lidar_now;
  world_to_body_now_ = world_to_lidar_now * body_to_lidar_.inverse();

  //bot_core::pose_t pose_msg = getPoseAsBotPose( world_to_lidar_now , msg->utime);
  //lcm_->publish("POSE_BODY_ALT", &pose_msg );
  bot_core::pose_t pose_msg_body = getPoseAsBotPose( world_to_body_now_ , msg->utime);
  lcm_->publish(cl_cfg_.output_channel, &pose_msg_body );
}


void App::initState(const double trans[3], const double quat[4]){
  if ( !cl_cfg_.init_with_message || pose_initialized_ ){
    return;
  }

  std::cout << "Init internal est using rigid transform or pose\n";
  
  world_to_body_init_.setIdentity();
  world_to_body_init_.translation()  << trans[0], trans[1] , trans[2];
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quat[0], quat[1], 
                                               quat[2], quat[3]);
  world_to_body_init_.rotate(quatE); 
  pose_initialized_ = TRUE;
}

void App::rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  initState(msg->trans, msg->quat);
}

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  initState(msg->pos, msg->orientation);
}

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.use_velodyne = false;
  cl_cfg.init_with_message = TRUE;
  cl_cfg.output_channel = "POSE_BODY";

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.init_with_message, "g", "init_with_message", "Bootstrap internal estimate using VICON or POSE_INIT");
  parser.add(cl_cfg.output_channel, "o", "output_channel", "Output message e.g POSE_BODY");
  parser.add(cl_cfg.use_velodyne, "v", "use_velodyne", "Use a velodyne instead of the LIDAR");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App app= App(lcm, cl_cfg);
  while(0 == lcm->handle());
}

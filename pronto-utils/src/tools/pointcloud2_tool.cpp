#include <lcm/lcm-cpp.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <path_util/path_util.h>
#include <laser_utils/laser_util.h>

#include <lcmtypes/pronto/pointcloud2_t.hpp>
#include <lcmtypes/pronto/pointcloud_t.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <pronto_utils/conversions_lcm.hpp>

#include <ConciseArgs>


namespace pcl
{
  // Euclidean Velodyne coordinate, including intensity and ring number. 
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

};

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))


struct CommandLineConfig
{
  bool verbose;
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

    pronto_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;

    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::pointcloud2_t* msg);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       cl_cfg_(cl_cfg_), lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_)
{

  lcm_recv_->subscribe("VELODYNE",&App::viconHandler,this);
  cout <<"App Constructed\n";

  bool reset = 1;
  pc_vis_ = new pronto_vis( lcm_pub_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70000,"Pose - Velodyne",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(70001,"Cloud - Velodyne"         ,1,reset, 70000,1, {0.0, 0.0, 1.0} ));

  botparam_ = bot_param_new_from_server(lcm_recv_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_recv_->getUnderlyingLCM(), botparam_);

}

int get_trans_with_utime(BotFrames *bot_frames,
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

void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::pointcloud2_t* msg){

  // 1. convert to a pcl point cloud:
  pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZIR> ());
  pcl::fromLCMPointCloud2( *msg, *cloud);
  //pcl::io::savePCDFileASCII ("mm.pcd", *cloud);


  // 2. republish the velodyne as a collections message (on the velodyne frame
  Eigen::Isometry3d velodyne_to_local;
  get_trans_with_utime( botframes_ , "VELODYNE", "local"  , msg->utime, velodyne_to_local);
  Isometry3dTime velodyne_to_local_T = Isometry3dTime(msg->utime, velodyne_to_local);
  pc_vis_->pose_to_lcm_from_list(70000, velodyne_to_local_T);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::copyPointCloud(*cloud, *cloud_xyzrgb);
  pc_vis_->ptcld_to_lcm_from_list(70001, *cloud_xyzrgb, msg->utime, msg->utime);  

  // 3. Find the horizontal lidar beam - could use the ring value but PCL filter doesnt handle it
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::copyPointCloud(*cloud, *cloud_xyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_xyz);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.01);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  //pcl::io::savePCDFileASCII ("mm_filtered.pcd", *cloud_filtered);


  // 4. republish beam as new message
  pronto::pointcloud_t out;
  out.utime = msg->utime;
  out.seq = msg->seq;
  out.frame_id = msg->frame_id;

  for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
    std::vector<float> point = { cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z};
    out.points.push_back(point);
  }
  out.n_points = out.points.size();
  out.n_channels = 0;
//  lcm_pub_->publish("VELODYNE_HORIZONTAL",&out);

}





int main(int argc, char **argv){
  CommandLineConfig cl_cfg;

  ConciseArgs parser(argc, argv, "fovision-odometry");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);

  App fo= App(lcm_recv, lcm_pub, cl_cfg);
  while(0 == lcm_recv->handle());
}

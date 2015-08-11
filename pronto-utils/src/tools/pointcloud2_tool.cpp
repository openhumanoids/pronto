#include <lcm/lcm-cpp.hpp>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <lcmtypes/drc/pointcloud2_t.hpp>
#include <lcmtypes/drc/pointcloud_t.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <ConciseArgs>

#include <path_util/path_util.h>

#include <pronto_utils/conversions_lcm.hpp>



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

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_);
    
    ~StereoOdom(){
    }

  private:
    const CommandLineConfig cl_cfg_;    
    
    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;

    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::pointcloud2_t* msg);
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       cl_cfg_(cl_cfg_), lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_)
{

  lcm_recv_->subscribe("VELODYNE",&StereoOdom::viconHandler,this);
  cout <<"StereoOdom Constructed\n";
}



void StereoOdom::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::pointcloud2_t* msg){
   cout <<"got vdyne\n";

  pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZIR> ());
  pcl::fromLCMPointCloud2( *msg, *cloud);
  //pcl::io::savePCDFileASCII ("mm.pcd", *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  // Find the horizontal lidar beam - could use the ring value but PCL filter doesnt handle it
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_xyz);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.01);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  //pcl::io::savePCDFileASCII ("mm_filtered.pcd", *cloud_filtered);

  // republish beam as new message
  drc::pointcloud_t out;
  out.utime = msg->utime;
  out.seq = msg->seq;
  out.frame_id = msg->frame_id;

  for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
    std::vector<float> point = { cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z};
    out.points.push_back(point);
  }
  out.n_points = out.points.size();
  out.n_channels = 0;
  lcm_pub_->publish("VELODYNE_HORIZONTAL",&out);

}





int main(int argc, char **argv){
  CommandLineConfig cl_cfg;

  ConciseArgs parser(argc, argv, "fovision-odometry");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);

  StereoOdom fo= StereoOdom(lcm_recv, lcm_pub, cl_cfg);
  while(0 == lcm_recv->handle());
}

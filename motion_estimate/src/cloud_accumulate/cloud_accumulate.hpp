#ifndef CLOUD_ACCUMULATE_HPP_
#define CLOUD_ACCUMULATE_HPP_

#include <deque>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>
#include <lcmtypes/bot_core/pointcloud_t.hpp>

#include <laser_utils/laser_util.h>
#include <path_util/path_util.h>

//#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pronto_utils/pronto_vis.hpp>
#include <pronto_utils/pronto_lcm.hpp> // decode perception lcm messages
#include <pronto_utils/pronto_frame_check_tools.hpp>

#include <boost/circular_buffer.hpp>
////////////////////////////////////////
struct CloudAccumulateConfig
{
    int batch_size;
    std::string lidar_channel;
    double max_range;
    double min_range;
};

class CloudAccumulate{
  public:
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_);
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_,
                    BotParam* botparam, BotFrames* botframes);

    ~CloudAccumulate(){
      delete laser_projector_;
      delete projected_laser_scan_;
    }
    
    int getCounter(){ return counter_; }
    bool getFinished(){ return finished_; }
    long getFinishedTime() { return utimeFinished_; }
    pronto::PointCloud* getCloud(){ return combined_cloud_; }
    Laser_projector* getLaserProjector(){ return laser_projector_; }
    
    void clearCloud(){ 
      combined_cloud_->points.clear(); 
      counter_ = 0;
      finished_ = false;
      //std::cout << "Empty previous map\n";
    }
    
    void publishCloud(pronto::PointCloud* &cloud);
    void writePCD(std::string filename, pronto::PointCloud* &cloud){
      pc_vis_->writePCD(filename, *cloud);
    }

    // returns true is scan was added, false if not
    bool processLidar(const bot_core::planar_lidar_t* msg);

    bool processLidar(std::shared_ptr<bot_core::planar_lidar_t> msg);

    bool processPointcloud2(const bot_core::pointcloud2_t* msg);

    bool processPointcloud(std::shared_ptr<bot_core::pointcloud_t> msg);
    bool processPointcloud(bot_core::pointcloud_t* msg);

    bool processPCLPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int64_t &utime);

    bool processProntoPointcloud(pronto::PointCloud* scan_velodyne, const int64_t &utime);
  private:
    void init(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_,
                          BotParam* botparam, BotFrames* botframes);

    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;
    
    pronto_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    FrameCheckTools frame_check_tools_;
    
    int counter_; 
    int verbose_;
    
    // Create a circular buffer
    boost::circular_buffer<std::shared_ptr<bot_core::planar_lidar_t>> messages_buffer_;
    
    pronto::PointCloud* combined_cloud_;
    
    bool finished_;
    int64_t utimeFinished_;
    
    Laser_projector * laser_projector_;
    laser_projected_scan * projected_laser_scan_;  
    
    // Project an individual lidar scan into the local/world frame
    pronto::PointCloud* convertPlanarScanToCloud(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    
};


#endif

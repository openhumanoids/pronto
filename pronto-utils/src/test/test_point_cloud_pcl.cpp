#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lcm/lcm-cpp.hpp>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " 
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd"
            << std::endl;


  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  pronto_vis* pc_vis_;
  pc_vis_ = new pronto_vis( lcm->getUnderlyingLCM() );

  int cfg_root = 1;
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  Isometry3dTime poseT = Isometry3dTime ( 0, pose  );
  obj_cfg oconfig = obj_cfg(cfg_root,   "Pose"  ,5,1);
  pc_vis_->pose_to_lcm(oconfig,poseT);

  ptcld_cfg pconfig = ptcld_cfg(cfg_root+1,  "Cloud"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig, *cloud, 0, 0);  

  
  pronto::PointCloud* cloud2 (new pronto::PointCloud);
  pronto::Point pt;
  pt.x =1; pt.y=0; pt.z = 2;
  cloud2->points.push_back(pt);

  std::cout << "Pronto Points: " << cloud2->points.size() << "\n";

  ptcld_cfg pconfig2 = ptcld_cfg(cfg_root+2,  "Cloud2"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig2, *cloud2, 0, 0);  

  pronto::PointCloud* cloud3 (new pronto::PointCloud);
  pc_vis_->convertCloudPclToPronto(*cloud,*cloud3);
  std::cout << "Pronto Points: " << cloud3->points.size() << "\n";  
  ptcld_cfg pconfig3 = ptcld_cfg(cfg_root+3,  "Cloud3"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig3, *cloud3, 0, 0);  


  return (0);
}

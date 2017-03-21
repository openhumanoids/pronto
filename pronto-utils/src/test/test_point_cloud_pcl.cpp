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
  //pose.translation().x() = -2.0; // offset the rendering
  Isometry3dTime poseT = Isometry3dTime ( 0, pose  );
  obj_cfg oconfig = obj_cfg(cfg_root,   "Pose"  ,5,1);
  pc_vis_->pose_to_lcm(oconfig,poseT);

  ptcld_cfg pconfig = ptcld_cfg(cfg_root+1,  "Cloud - pcl"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig, *cloud, 0, 0);  

  
  pronto::PointCloud* cloud2 (new pronto::PointCloud);
  pronto::Point pt;
  pt.x =1; pt.y=0; pt.z = 2;
  cloud2->points.push_back(pt);

  std::cout << "Pronto Points: " << cloud2->points.size() << "\n";

  ptcld_cfg pconfig2 = ptcld_cfg(cfg_root+2,  "Cloud2 - pronto"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig2, *cloud2, 0, 0);  

  pronto::PointCloud* cloud3 (new pronto::PointCloud);
  pc_vis_->convertCloudPclToPronto(*cloud,*cloud3);
  std::cout << "Pronto Points: " << cloud3->points.size() << "\n";  
  ptcld_cfg pconfig3 = ptcld_cfg(cfg_root+3,  "Cloud3 - pronto"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig3, *cloud3, 0, 0);  


  // Test 2: Transformation of pointclouds:
  Eigen::Isometry3d pose_transform = Eigen::Isometry3d::Identity();
  pose_transform.translation().x() = 0.5;
  pose_transform.translation().y() = 1;

  // a: pcl method:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*cloud, *cloud4,
                           pose_transform.translation().cast<float>(), Eigen::Quaternionf(pose_transform.rotation().cast<float>()));
  ptcld_cfg pconfig4 = ptcld_cfg(cfg_root+4,  "Cloud4 - pcl"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig4, *cloud4, 0, 0);

  // b: pronto method:
  pronto::PointCloud* cloud5 (new pronto::PointCloud);
  pc_vis_->transformPointCloud(*cloud3, *cloud5, Eigen::Affine3f ( pose_transform.cast<float>() ) );
  ptcld_cfg pconfig5 = ptcld_cfg(cfg_root+5,  "Cloud5 - pronto"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig5, *cloud5, 0, 0);


  sleep(2.0);
  std::cout << "Sleep two seconds, then move root pose\n";
  std::cout << "which indirectly causes the point clouds to move\n";

  pose.translation().x() = 1.0;
  poseT = Isometry3dTime ( 0, pose  );
  oconfig = obj_cfg(cfg_root,   "Pose"  ,5,1); 
  pc_vis_->pose_to_lcm(oconfig,poseT);


  return (0);
}

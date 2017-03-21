// Demo program for collections and pronto_vis
// 1 create some poses, create some point clouds
// 2 then publish the poses and publish the point clouds - to be attached relative to the poses
// 

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <lcm/lcm-cpp.hpp>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds

int
main (int argc, char** argv)
{

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  pronto_vis* pc_vis_;
  pc_vis_ = new pronto_vis( lcm->getUnderlyingLCM() );


  // obj cfg: id name type reset
  // pts cfg: id name type reset objcoll usergb rgb


  // Create some poses
  Isometry3dTime poseT_0 = Isometry3dTime(0,Eigen::Isometry3d::Identity());
  poseT_0.pose.translation().x() = 0.5;

  Isometry3dTime poseT_1 = Isometry3dTime(1,Eigen::Isometry3d::Identity());
  poseT_1.pose.translation().x() = 1.5;

  Isometry3dTime poseT_2 = Isometry3dTime(2,Eigen::Isometry3d::Identity());
  poseT_2.pose.translation().x() = 2.5;

  std::vector<Isometry3dTime> posesT;
  posesT.push_back( poseT_0 );
  posesT.push_back( poseT_1 );
  posesT.push_back( poseT_2 );

  // Send a list of poses/objects as OBJECT_COLLECTION
  int obj_coll_id=1;
  obj_cfg oconfig = obj_cfg(obj_coll_id,   "Poses"  ,5,1);
  pc_vis_->pose_collection_to_lcm(oconfig, posesT);


  // These would typically be int64_t timestamps:
  std::vector<int64_t> obj_ids;
  obj_ids.push_back(0);
  //  obj_ids.push_back(1);
  obj_ids.push_back(2);

  std::vector<int64_t> ptcld_ids;
  ptcld_ids.push_back(0);
  //  ptcld_ids.push_back(1);
  ptcld_ids.push_back(2);


  // Create some "point clouds"
  pcl::PointCloud<pcl::PointXYZRGB> cloud0;
  pcl::PointXYZRGB pt;
  pt.x =0; pt.y=1; pt.z = 0;   pt.r =0; pt.g=0; pt.b = 255; 
  cloud0.points.push_back(pt);

  //pcl::PointCloud<pcl::PointXYZRGB> cloud1;
  //pt.x =0; pt.y=2; pt.z = 0; pt.r =255; pt.g=0; pt.b = 0; 
  //cloud1.points.push_back(pt);

  pcl::PointCloud<pcl::PointXYZRGB> cloud2;
  pt.x =0; pt.y=3; pt.z = 0; pt.r =0; pt.g=255; pt.b = 0; 
  cloud2.points.push_back(pt);

  std::vector< pcl::PointCloud<pcl::PointXYZRGB> > clouds;
  clouds.push_back(cloud0);
  //clouds.push_back(cloud1);
  clouds.push_back(cloud2);

  // Send the collection of point cloud lists
  ptcld_cfg pconfig = ptcld_cfg(2,  "Clouds"     ,1,1, obj_coll_id,0, {0.2,0,0.2} );
  pc_vis_->ptcld_collection_to_lcm(pconfig, clouds, obj_ids, ptcld_ids);  


  sleep(2.0);

  std::cout << "Sleep two seconds, then change the point cloud list\n";
  std::cout << "attached to pose 1 (only)\n\n";
  pcl::PointCloud<pcl::PointXYZRGB> cloud4;
  pt.x =0; pt.y=-2; pt.z = 0; pt.r =255; pt.g=255; pt.b = 0; 
  cloud4.points.push_back(pt);
  pconfig.reset = 0;
  pc_vis_->ptcld_to_lcm(pconfig, cloud4, 1, 1);


  sleep(2.0);
  std::cout << "Sleep two seconds, then change pose 2\n";
  std::cout << "(which will move the point cloud implicitly\n\n";
  Isometry3dTime poseT_4 = Isometry3dTime(2,Eigen::Isometry3d::Identity());
  poseT_4.pose.translation().x() = 5.0;
  oconfig.reset = 0;
  pc_vis_->pose_to_lcm(oconfig, poseT_4);


  return (0);
}

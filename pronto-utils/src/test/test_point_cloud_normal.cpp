// Demo program for visualising normals in director and pronto viewer

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <lcm/lcm-cpp.hpp>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds

#include <pcl/features/normal_3d.h>


// radious of .5m seems good
// pcl_viewer test.pcd  -normals 1 -normals_scale 0.1


int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("pronto_vis_normals.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " 
            << cloud->width * cloud->height
            << " data points from pronto_vis_normals.pcd"
            << std::endl;


  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 50cm
  ne.setRadiusSearch (0.5);

  // Compute the features
  ne.compute (*cloud_normals);


  // Combine normals with points:
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>); 
  pcl::copyPointCloud(*cloud, *cloud_with_normals); 
  pcl::copyPointCloud(*cloud_normals, *cloud_with_normals); 
  //pcl::io::savePCDFileASCII("test.pcd", *cloud_with_normals);


  // Create normal list:
  std::vector<Eigen::Vector3d> normal_list;
  for (size_t i =0; i< cloud_with_normals->points.size() ;i++){
    Eigen::Vector4f p = cloud_with_normals->points[i].getNormalVector4fMap (); 
    normal_list.push_back ( Eigen::Vector3d( p[0],p[1],p[2] ) ) ; 
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  pronto_vis* pc_vis_;
  pc_vis_ = new pronto_vis( lcm->getUnderlyingLCM() );

  int cfg_root = 1;
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  Isometry3dTime poseT = Isometry3dTime ( 0, pose  );
  obj_cfg oconfig = obj_cfg(cfg_root,   "Pose"  ,5,1);
  pc_vis_->pose_to_lcm(oconfig,poseT);

  // As a PCL point cloud
  ptcld_cfg pconfig = ptcld_cfg(cfg_root+1,  "Cloud with normals - pcl"     ,1,1, cfg_root,1, {0.1,0.7,0.1} );
  pc_vis_->ptcld_to_lcm(pconfig, *cloud, normal_list, 0, 0);  

  // As a Pronto point cloud
  pronto::PointCloud* cloud3 (new pronto::PointCloud);
  pc_vis_->convertCloudPclToPronto(*cloud,*cloud3); 
  ptcld_cfg pconfig2 = ptcld_cfg(cfg_root+2,  "Cloud with normals - pronto"     ,1,1, cfg_root,1, {0.7,0.1,0.1} );
  pc_vis_->ptcld_to_lcm(pconfig2, *cloud3, normal_list, 0, 0);  


  return (0);
}

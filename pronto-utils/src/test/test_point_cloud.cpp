#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <boost/shared_ptr.hpp>


int
main (int argc, char** argv)
{

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  pronto_vis* pc_vis_;
  pc_vis_ = new pronto_vis( lcm->getUnderlyingLCM() );

  int cfg_root = 1;
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  Isometry3dTime poseT = Isometry3dTime ( 0, pose  );
  obj_cfg oconfig = obj_cfg(cfg_root,   "Pose"  ,5,1);
  pc_vis_->pose_to_lcm(oconfig,poseT);

  pronto::PointCloud* cloud2 (new pronto::PointCloud);
  pronto::Point pt;
  pt.x =1; pt.y=0; pt.z = 2;
  cloud2->points.push_back(pt);

  std::cout << "Pronto Points: " << cloud2->points.size() << "\n";

  ptcld_cfg pconfig2 = ptcld_cfg(cfg_root+2,  "Cloud2"     ,1,1, cfg_root,0, {0.2,0,0.2} );
  pc_vis_->ptcld_to_lcm(pconfig2, *cloud2, 0, 0);  



  return (0);
}

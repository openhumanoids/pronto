#include <iostream>
#include <lcm/lcm.h>

#include "visualization/collections_math.hpp"
#include "visualization/viewer.hpp"
#include "visualization/pointcloud.hpp"

using namespace std;

int main(int argc, char** argv)
{
  lcm_t * lcm;

  lcm = lcm_create(NULL);
  if(!lcm)
  {
    cerr << "Failed to create lcm connection" << endl;
    return 1;
  }

  cout << "Collections example" << endl;
  Viewer viewer(lcm);
  ObjectCollection obj(1, std::string("Objects"), VS_OBJECT_COLLECTION_T_POSE3D);
  LinkCollection link(2, std::string("Links"));

  // Send a reset
  // Send an object collection
  obj.add(1, coll::Pose3d(0,0,0,0,0,0));
  obj.add(101, coll::Pose3d(10,0,0,0,0,0));
  obj.add(102, coll::Pose3d(10,10,0,0,0,0));
  obj.add(103, coll::Pose3d(0,10,0,0,0,0));
  viewer.sendCollection(obj, true);

  // Send a link collection
  link.add(1, 1, 1, 1, 101);
  link.add(2, 1, 102, 1, 103);
  viewer.sendCollection(link, true);

  // Send a point collection
  PointCloudCollection pc_collection(3, std::string("Point cloud"), 1, VS_POINT3D_LIST_COLLECTION_T_POINT);
  PointCloudPtr pc = boost::make_shared<PointCloud>(1);
  pc->addPoint( 1.,  1.,  1., 1., 0., 0.);
  pc->addPoint( 0.,  1.,  1.1, 0., 1., 0.);
  pc->addPoint( 0.,  -1.,  1.2, 0., 0., 1.);
  pc_collection.add(boost::make_shared<PointCloudPtr>(pc));
  viewer.sendCollection(pc_collection, true);
  std::cout << "Send point cloud" << std::endl;

//sleep(2);
{
  ObjectCollection obj(4, std::string("Objects 2"), VS_OBJECT_COLLECTION_T_AXIS3D);
  // Send an object collection
  obj.add(2010, coll::Pose3d(-2,-2,0,1.571,0,0));
  obj.add(2011, coll::Pose3d(-3,-2,0,1.571,0,0));
  obj.add(2012, coll::Pose3d(-3,-3,0,1.571,0,0));
  viewer.sendCollection(obj, true);


  PointCloudCollection pc_collection(5, std::string("Point cloud 2"), 4, VS_POINT3D_LIST_COLLECTION_T_POINT);
  PointCloudPtr pc = boost::make_shared<PointCloud>(2000);
  pc->addPoint( 0.2,  0.1,  0.);
  pc->addPoint( -0.2, 0.1,  0.);
  pc->addPoint( 0.2, -0.1,  0.);
  pc_collection.add(boost::make_shared<PointCloudPtr>(pc));
  viewer.sendCollection(pc_collection, true);
  std::cout << "Send point cloud" << std::endl;
}
//sleep(2);
{
  ObjectCollection obj(8, std::string("Objects 3"), VS_OBJECT_COLLECTION_T_POSE3D);
  // Send an object collection
  obj.add(200, coll::Pose3d(01,15,0,0,0,0));
  obj.add(201, coll::Pose3d(11,15,0,0,0,0));
  obj.add(202, coll::Pose3d(11,25,0,0,0,0));
  obj.add(203, coll::Pose3d(1,25,0,0,0,0));
  viewer.sendCollection(obj, true);


  PointCloudCollection pc_collection(7, std::string("Point cloud 3"), 8, VS_POINT3D_LIST_COLLECTION_T_TRIANGLES);
  PointCloudPtr pc = boost::make_shared<PointCloud>(200);
  pc->addPoint(11.,  1.,  0.);
  pc->addPoint( 2., 10.,  0.);
  pc->addPoint( 2.,  1.,  0.);
  pc_collection.add(boost::make_shared<PointCloudPtr>(pc));
  viewer.sendCollection(pc_collection, true);
  std::cout << "Send point cloud" << std::endl;
}

sleep(2);
  std::cout << "Move the first Objects poses" << std::endl;


{
  ObjectCollection obj(1, std::string("Objects"), VS_OBJECT_COLLECTION_T_POSE3D);


  // Send a reset
  // Send an object collection
  obj.add(1, coll::Pose3d(5,0,0,0,0,0));
  obj.add(101, coll::Pose3d(15,0,0,0,0,0));
  obj.add(102, coll::Pose3d(15,10,0,0,0,0));
  obj.add(103, coll::Pose3d(5,10,0,0,0,0));
  viewer.sendCollection(obj, true);
}


  /*
  sleep(2);
  std::cout << "Reset the collections ... (i.e. delete the above information" << std::endl;
  vs_reset_collections_t r;
  vs_reset_collections_t_publish(lcm, "RESET_COLLECTIONS", &r);
  */

  lcm_destroy(lcm);
  return 0;
}


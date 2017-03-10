#ifndef PRONTO_VIS_HPP_
#define PRONTO_VIS_HPP_

#include <lcm/lcm.h>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/assign/std/vector.hpp>

#include "pronto_vis_config.h"

#ifdef USE_PRONTO_VIS_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"

#include <pcl/filters/passthrough.h>
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/concave_hull.h"
#include "pcl/PolygonMesh.h"
#include "pcl/octree/octree.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/ia_ransac.h"
#endif

#include <vector>
#include <algorithm>

#include <pronto_utils/pronto_math.hpp>


namespace pronto
{
  struct Vertices
  {
    Vertices () : vertices ()
    {}

    std::vector<uint32_t> vertices;
  };

  struct Point
  {
    Point () {}

    float x,y,z;
    uint8_t r,g,b; // 0-255
  };

  struct PointCloud
  {
    PointCloud () : points ()
    {}

    std::vector< pronto::Point > points;
  };

  struct PolygonMesh
  {
    PolygonMesh () : cloud () , polygons()
    {}
    pronto::PointCloud cloud;
    std::vector< pronto::Vertices > polygons;
  };

}



class pronto_vis {
  public:
    pronto_vis (lcm_t* publish_lcm);

    std::vector <obj_cfg> obj_cfg_list;
    std::vector <ptcld_cfg> ptcld_cfg_list;

    // Duplicates the list in collections renderer:
    // assumed to be 3xN colors
    std::vector < float > colors;    

    ///////////// Text and Pose Methods //////////////////////////
    // Push a colour PointCloud to LCM as a points collection
    // assumes that you want to connect it to the collection specified in Ptcoll_cfg
    void text_collection_to_lcm(int text_collection_id,
                            int object_collection_id, std::string text_collection_name,
                            std::vector<std::string >& labels, 
                            std::vector<int64_t>& object_ids);

    void link_collection_to_lcm(link_cfg lcfg, std::vector<link_data> ldata);
    void link_to_lcm(link_cfg lcfg, link_data ldata);
    
    void pose_collection_to_lcm_from_list(int id, std::vector<Isometry3dTime> & posesT);
    void pose_collection_to_lcm(obj_cfg ocfg, std::vector<Isometry3dTime> & posesT);

    // Also can be used by a single pose
    void pose_collection_reset(int id, std::string name);


    void pose_to_lcm_from_list(int id,Isometry3dTime& poseT);
    void pose_to_lcm(obj_cfg ocfg, Isometry3dTime& poseT);

    void ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pronto::PointCloud > &clouds,
            int64_t obj_id, int64_t ptcld_id);

    void ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pronto::PointCloud > &clouds,
            std::vector<int64_t> &obj_ids, std::vector<int64_t> &ptcld_ids);

    void ptcld_to_lcm_from_list(int id, pronto::PointCloud &cloud,
            int64_t obj_id, int64_t ptcld_id);
    void ptcld_to_lcm(ptcld_cfg pcfg, pronto::PointCloud &cloud,
            int64_t obj_id, int64_t ptcld_id);
    // Also can be used by mesh
    void ptcld_collection_reset(int id, std::string name);

    // Equivalent to PCL function of same name:
    // Conversion from Isometry3d to Affine3f:
    // Eigen::Affine3f t ( pose_transform.cast<float>() );
    void transformPointCloud(pronto::PointCloud &cloud_in, pronto::PointCloud &cloud_out, Eigen::Affine3f transform);

    // Duplicates a function in drc_lidarUtils
    // should this be recoded to use pronto::PointCloud?
    static bool interpolateScan(const std::vector<float>& iRanges,
                  const double iTheta0, const double iThetaStep,
                  const Eigen::Isometry3d& iPose0,
                  const Eigen::Isometry3d& iPose1,
                  std::vector<Eigen::Vector3f>& oPoints);


    void convertLidar(std::vector< float > ranges, int numPoints, double thetaStart,
        double thetaStep,
        pronto::PointCloud* &cloud,
	double minRange = 0., double maxRange = 1e10,
	double validRangeStart = -1000, double validRangeEnd = 1000);

    // Very basic implementation of PCL's ascii pcd writer
    void writePCD(std::string filename, pronto::PointCloud &cloud);

    ///////////// Point Cloud Methods ////////////////////////////
#ifdef USE_PRONTO_VIS_PCL
    void convertCloudPclToPronto(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pronto::PointCloud &cloud_out);
    void convertCloudProntoToPcl(pronto::PointCloud &cloud, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
    // Plot a set of point clouds relative to a specific pose
    void ptcld_collection_to_lcm_from_list(int id, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
            int64_t obj_id, int64_t ptcld_id);
    void ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
        std::vector<int64_t> &obj_ids, std::vector<int64_t> &ptcld_ids); 
    void ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
            int64_t obj_id, int64_t ptcld_id);
    
    void ptcld_to_lcm_from_list(int id, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            int64_t obj_id, int64_t ptcld_id);
    void ptcld_to_lcm(ptcld_cfg pcfg, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            int64_t obj_id, int64_t ptcld_id);

    void mesh_to_lcm_from_list(int id, pcl::PolygonMesh::Ptr mesh,
            int64_t obj_id, int64_t ptcld_id,
            bool sendSubset =false,const std::vector<int> &SubsetIndicies = std::vector<int>());
    void mesh_to_lcm(ptcld_cfg pcfg,pcl::PolygonMesh::Ptr mesh,
            int64_t obj_id, int64_t ptcld_id,
            bool sendSubset =false,const std::vector<int> &SubsetIndicies = std::vector<int>());



    // Merge two PolygonMesh structures into the meshA
    bool mergePolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB);
    
    void ptcldToOctomapLogFile(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            std::string filename);

    // Find the polygon INDICES inside a box centered around @center of 2x size @dgrid
    // of the PolygonMEsh meshin_ptr
    // @input: meshin_ptr - input mesh model
    // @input: center - 3 element vector to center of box
    // @input: dgrid - 3 elements of size of boxlcm
    // @output: the polygon INDICES inside the box
    void getMeshInBoxIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, std::vector<double> &dgrid,
		   std::vector<int> &polygon_in_box_indices);

    // Find the polygon INDICES inside a circle of size @radius
    // of the PolygonMEsh meshin_ptr
    // @input: meshin_ptr - input mesh model
    // @input: center - 3 element vector to center of box
    // @input: radius - 3 elements of size of box
    // @output: the polygon INDICES inside the box
    void getMeshInCircleIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, double radius,
		   std::vector<int> &polygon_in_box_indices);
    
    // Remove all the polygons with a specific color
    // this is used to remove all black polygons which represent glass in kcml
    void removeColoredPolygons(pcl::PolygonMesh::Ptr meshin_ptr, std::vector<int> &color );

    // Save a PolygonMesh to PLY
    // PCL's uses VTK which produces an awkward type of PLY
    void savePLYFile(pcl::PolygonMesh::Ptr model,std::string fname);

    // Find the polygons inside a box centered around @center of 2x size @dgrid
    // of the PolygonMEsh meshin_ptr
    // @input: meshin_ptr - input mesh model
    // @input: center - 3 element vector to center of box
    // @input: dgrid - 3 elements of size of box
    // @output: what lies insdie the box
    void getMeshInBox(pcl::PolygonMesh::Ptr meshin_ptr,
		   std::vector<double> &center, std::vector<double> &dgrid,
		   pcl::PolygonMesh::Ptr &minimesh_ptr);
#endif

  private:
    lcm_t *publish_lcm_;

};


// Read a csv file containing poses and timestamps:
void read_poses_csv(std::string poses_files, std::vector<Isometry3dTime>& poses);
void display_tic_toc(std::vector<int64_t> &tic_toc,const std::string &fun_name);

// http://www.alecjacobson.com/weblog/?p=1527
// Acts like matlab's [Y,I] = SORT(X)
// Input:
//   unsorted  unsorted vector
// Output:
//   sorted     sorted vector, allowed to be same as unsorted
//   index_map  an index map such that sorted[i] = unsorted[index_map[i]]
template <class T>
void sort(
    std::vector<T> &unsorted,
    std::vector<T> &sorted,
    std::vector<size_t> &index_map);

// Act like matlab's Y = X[I]
// where I contains a vector of indices so that after,
// Y[j] = X[I[j]] for index j
// this implies that Y.size() == I.size()
// X and Y are allowed to be the same reference
template< class T >
void reorder(
  std::vector<T> & unordered,
  std::vector<size_t> const & index_map,
  std::vector<T> & ordered);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

// Comparison struct used by sort
// http://bytes.com/topic/c/answers/132045-sort-get-index
template<class T> struct index_cmp
{
  index_cmp(const T arr) : arr(arr) {}
  bool operator()(const size_t a, const size_t b) const
  {
    return arr[a] < arr[b];
  }
  const T arr;
};

template <class T>
void sort(
  std::vector<T> & unsorted,
  std::vector<T> & sorted,
  std::vector<size_t> & index_map)
{
  // Original unsorted index map
  index_map.resize(unsorted.size());
  for(size_t i=0;i<unsorted.size();i++)
  {
    index_map[i] = i;
  }
  // Sort the index map, using unsorted for comparison
  sort(
    index_map.begin(),
    index_map.end(),
    index_cmp<std::vector<T>& >(unsorted));

  sorted.resize(unsorted.size());
  reorder(unsorted,index_map,sorted);
}

// This implementation is O(n), but also uses O(n) extra memory
template< class T >
void reorder(
  std::vector<T> & unordered,
  std::vector<size_t> const & index_map,
  std::vector<T> & ordered)
{
  // copy for the reorder according to index_map, because unsorted may also be
  // sorted
  std::vector<T> copy = unordered;
  ordered.resize(index_map.size());
  for(size_t i = 0; i<index_map.size();i++)
  {
    ordered[i] = copy[index_map[i]];
  }
}


#endif /* PRONTO_VIS_HPP_ */

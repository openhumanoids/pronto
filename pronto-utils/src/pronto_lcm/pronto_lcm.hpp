// This class has be continously emptied and now contains
// practically no contents. It can probably be removed
// if the convertLidar methods are not used

#ifndef PRONTO_LCM_HPP_
#define PRONTO_LCM_HPP_

#include <iostream>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// this seems to be required. don't know why?
#include "pcl/kdtree/kdtree_flann.h"

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

using namespace pcl;
using namespace pcl::io;

class pronto_lcm {
  public:
    pronto_lcm (lcm_t* publish_lcm);

  private:
    lcm_t *publish_lcm_; 
};


inline void
convertLidar(const float * ranges, int numPoints, double thetaStart,
        double thetaStep,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
	double minRange = 0., double maxRange = 1e10,
	double validRangeStart = -1000, double validRangeEnd = 1000)
{
  int count = 0;
  double theta = thetaStart;

  cloud->width   = numPoints;
  cloud->height   = 1;
  cloud->points.resize (numPoints);

  // minRange was 0.1 until march 2013
  for (int i = 0; i < numPoints; i++) {
      if (ranges[i] > minRange && ranges[i] < maxRange && theta > validRangeStart
              && theta < validRangeEnd) { 
          //hokuyo driver seems to report maxRanges as .001 :-/
          //project to body centered coordinates
          cloud->points[count].x = ranges[i] * cos(theta);
          cloud->points[count].y = ranges[i] * sin(theta);
          cloud->points[count].z = 0;
          count++;
      }
      theta += thetaStep;
  }
  // Resize outgoing cloud
  cloud->width   = count;
  cloud->points.resize (count);
}


inline void
convertLidar(std::vector< float > ranges, int numPoints, double thetaStart,
        double thetaStep,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        double minRange = 0., double maxRange = 1e10,
        double validRangeStart = -1000, double validRangeEnd = 1000){
  int count = 0;
  double theta = thetaStart;

  cloud->width   = numPoints;
  cloud->height   = 1;
  cloud->points.resize (numPoints);

  // minRange was 0.1 until march 2013
  for (int i = 0; i < numPoints; i++) {
      if (ranges[i] > minRange && ranges[i] < maxRange && theta > validRangeStart
              && theta < validRangeEnd) { 
          //hokuyo driver seems to report maxRanges as .001 :-/
          //project to body centered coordinates
          cloud->points[count].x = ranges[i] * cos(theta);
          cloud->points[count].y = ranges[i] * sin(theta);
          count++;
      }
      theta += thetaStep;
  }
  // Resize outgoing cloud
  cloud->width   = count;
  cloud->points.resize (count);
}

#endif

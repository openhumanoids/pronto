#ifndef LIDAR_ODOM_HPP_
#define LIDAR_ODOM_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
// #include <lcmtypes/scanmatch.hpp>
// #include <scanmatch/ScanMatcher.hpp>
#include <frsm/frsm.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace frsm;

class LidarOdomConfig
{
  public:
    LidarOdomConfig();

    frsm_laser_type_t laserType;
    int beamSkip; //downsample ranges by only taking 1 out of every beam_skip points
    double spatialDecimationThresh; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
    double maxRange; //discard beams with reading further than this value
    double minRange; //discard beams with reading nearer than this value
    float validBeamAngles[2]; //valid part of the field of view of the laser in radians, 0 is the center beam
 
    //hardcoded scan matcher params
    double metersPerPixel; //translational resolution for the brute force search
    double thetaResolution; //angular step size for the brute force search
    frsm_incremental_matching_modes_t  matchingMode; //use gradient descent to improve estimate after brute force search
    int useMultires; // low resolution will have resolution metersPerPixel * 2^useMultiRes

    double initialSearchRangeXY; //nominal range that will be searched over
    double initialSearchRangeTheta;

    //SHOULD be set greater than the initialSearchRange
    double maxSearchRangeXY; //if a good match isn't found I'll expand and try again up to this size...
    double maxSearchRangeTheta; //if a good match isn't found I'll expand and try again up to this size...

    int maxNumScans; //keep around this many scans in the history
    double addScanHitThresh; //add a new scan to the map when the number of "hits" drops below this

    bool stationaryMotionModel; //use constant velocity model
    double motionModelPriorWeight; //don't use the prior for anything other than centering the window

    int useThreads;
    bool doDrawing;
  private:
};

class LidarOdom
{
public:
    LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_);
    LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_, LidarOdomConfig &cfg_);
    void init();
    
    ~LidarOdom();

    void doOdometry(float* ranges, int nranges, float rad0, float radstep, int64_t utime);
    void doOdometry(std::vector<float> x, std::vector<float> y, int npoints, int64_t utime);
    void doOdometry(std::vector<float> x, std::vector<float> y, int npoints, int64_t utime, ScanTransform* prior);

    Eigen::Isometry3d getCurrentPose(){  return currOdom_;  }

    Eigen::Isometry3d getMotion(){ return  (currOdom_ *  prevOdom_.inverse()); }

    void draw(frsmPoint * points, unsigned numPoints, const ScanTransform * T);

private:
    boost::shared_ptr<lcm::LCM> lcm_;
    LidarOdomConfig cfg_;

    ScanMatcher* sm_;
    bot_lcmgl_t * lcmgl_state_;
    bot_lcmgl_t * lcmgl_scan_;

    double lastDrawTime_;
    Eigen::Isometry3d prevOdom_, currOdom_;
    int64_t prevUtime_, currUtime_;
};

#endif

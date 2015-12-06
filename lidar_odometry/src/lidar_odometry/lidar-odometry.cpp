#include "lidar-odometry.hpp"

LidarOdomConfig::LidarOdomConfig(){
  laserType = FRSM_HOKUYO_UTM;
  beamSkip = 3;
  spatialDecimationThresh = .2;
  maxRange = 29.7;
  // Full 270 range for a UTM-30LX
  validBeamAngles[0]= -2.356194;
  validBeamAngles[1] = 2.356194;

  metersPerPixel = .02;
  thetaResolution = .01;
  matchingMode= FRSM_COORD_ONLY;
  useMultires = 3;

  initialSearchRangeXY = .15;
  initialSearchRangeTheta = .1;

  maxSearchRangeXY = .3;
  maxSearchRangeTheta = .2;

  maxNumScans = 30;
  addScanHitThresh = .80;

  stationaryMotionModel = false;
  motionModelPriorWeight =0;

  useThreads = 1;
  doDrawing = TRUE;
}

LidarOdom::LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_):
  lcm_(lcm_),cfg_(LidarOdomConfig()){
  init();
}

LidarOdom::LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_, LidarOdomConfig &cfg_):
  lcm_(lcm_), cfg_(cfg_){
  init();
}

void LidarOdom::init(){
  lastDrawTime_ =0;

  //create the actual scan matcher object
  sm_ = new ScanMatcher(cfg_.metersPerPixel, cfg_.thetaResolution, cfg_.useMultires,
            cfg_.useThreads,true);

  
  if (sm_->isUsingIPP())
      fprintf(stderr, "Using IPP\n");
  else
      fprintf(stderr, "NOT using IPP\n");
  

  currOdom_.setIdentity();
  prevOdom_.setIdentity();

  ScanTransform startPose;
  memset(&startPose, 0, sizeof(startPose));
  startPose.theta = (0 * M_PI)/180; //set the scan matcher to start at 0 heading... cuz pi/2 would be rediculous
  
  sm_->initSuccessiveMatchingParams(cfg_.maxNumScans, cfg_.initialSearchRangeXY,
            cfg_.maxSearchRangeXY, cfg_.initialSearchRangeTheta, cfg_.maxSearchRangeTheta,
            cfg_.matchingMode, cfg_.addScanHitThresh,
            cfg_.stationaryMotionModel,cfg_.motionModelPriorWeight,&startPose);

  if (cfg_.doDrawing) {
    lcmgl_state_ = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "FRSM_state");
    lcmgl_scan_ = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "FRSM_scan");
  }
  else {
    lcmgl_state_ = NULL;
    lcmgl_scan_ = NULL;
  }

  cout <<"LidarOdom Constructed\n";
}


LidarOdom::~LidarOdom()
{
}

void LidarOdom::draw(frsmPoint * points, unsigned numPoints, const ScanTransform * T)
{
  sm_->draw_state_lcmgl(lcmgl_state_);
  sm_->draw_scan_lcmgl(lcmgl_scan_, points, numPoints, T);
  bot_lcmgl_switch_buffer(lcmgl_scan_);
  bot_lcmgl_switch_buffer(lcmgl_state_);
}


Eigen::Isometry3d getScanTransformAsIsometry3d(ScanTransform tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.x, tf.y, 0;

  double rpy[3] = { 0, 0, tf.theta };
  double quat[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat);
  Eigen::Quaterniond q(quat[0], quat[1],quat[2],quat[3]);
  tf_out.rotate(q);

  return tf_out;
}


void LidarOdom::doOdometry(float* ranges, int nranges, float rad0, float radstep, int64_t utime){
    
    //Project ranges into points, and decimate points so we don't have too many
    frsmPoint * points = (frsmPoint *) calloc(nranges, sizeof(frsmPoint));
    int numValidPoints = frsm_projectRangesAndDecimate(cfg_.beamSkip,
            cfg_.spatialDecimationThresh, ranges, nranges, rad0,
            radstep, points, cfg_.maxRange, cfg_.validBeamAngles[0],
            cfg_.validBeamAngles[1]);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n",
                numValidPoints);
        free(points);
        return;
    }

    //Actually do the matching
    ScanTransform r = sm_->matchSuccessive(points, numValidPoints,
            cfg_.laserType, utime, NULL); //don't have a better estimate than prev, so just set prior to NULL
                                      //utime is ONLY used to tag the scans that get added to the map, doesn't actually matter
    Eigen::Isometry3d r_Iso = getScanTransformAsIsometry3d(r);
    prevOdom_ = currOdom_;
    currOdom_ = r_Iso;
    prevUtime_ = currUtime_;
    currUtime_ = utime;

    //Print current position periodically
    static double lastPrintTime = 0;
    if (frsm_get_time() - lastPrintTime > 2.0) {
        lastPrintTime = frsm_get_time();
        fprintf(stderr,
                "x=%+7.3f y=%+7.3f t=%+7.3f\t score=%f hits=%.2f sx=%.2f sxy=%.2f sy=%.2f st=%.2f, numValid=%d\n",
                r.x, r.y, r.theta, r.score, (double) r.hits / (double) numValidPoints, r.sigma[0], r.sigma[1],
                r.sigma[4], r.sigma[8], numValidPoints);
    }

    //Do drawing periodically
    // This was disabled when moving to the frsm library. it would be easily re-added on a fork
    //Do drawing periodically!
    if (cfg_.doDrawing && frsm_get_time() - lastDrawTime_ > .2) {
        lastDrawTime_ = frsm_get_time();
        //frsm_tictoc("drawing");
        draw(points, numValidPoints, &r);
        //frsm_tictoc("drawing");
    }

    //cleanup
    free(points);
}


/////// CODE BELOW IS TO ACCEPT A SET OF X,Y POINTS AND TREAT THEM AS LIDAR RETURNS
/////// ORIGINALLY USED FOR HORIZONTAL DATA COMING FROM A VELODYNE
int projectPointsToPoints(std::vector<float> x, std::vector<float> y, int numPoints,
    frsmPoint * points, double maxRange = 1e10, 
    double * aveValidRange = NULL, double * stddevValidRange = NULL)
{
  int count = 0;
  double aveRange = 0;
  double aveRangeSq = 0;

  for (int i = 0; i < numPoints; i++) {
    double range = sqrt( x[i]*x[i] + y[i]*y[i] );
    if (range > .1 && range < maxRange ) {
      points[count].x = x[i];
      points[count].y = y[i];
      count++;
      aveRange += range;
      aveRangeSq += frsm_sq(range);
    }
  }

  aveRange /= (double) count;
  aveRangeSq /= (double) count;

  if (aveValidRange != NULL)
    *aveValidRange = aveRange;
  if (stddevValidRange != NULL)
    *stddevValidRange = sqrt(aveRangeSq - frsm_sq(aveRange));

  return count;
}


int projectPointsAndDecimate(int beamskip, float spatialDecimationThresh, std::vector<float> x, std::vector<float> y, int npoints,
    frsmPoint * points, double maxRange = 1e10)
{
  int lastAdd = -1000;
  double aveRange;
  double stdDevRange;

  int numValidPoints = projectPointsToPoints(x, y, npoints,
    points, maxRange, &aveRange, &stdDevRange);

  frsmPoint origin = { 0, 0 };
  frsmPoint lastAddPoint = { 0, 0 };
  int numDecPoints = 0;
  for (int i = 0; i < numValidPoints; i++) {
    //add every beamSkip beam, unless points are more than spatialDecimationThresh, or more than 1.8 stdDevs more than ave range
    if ((i - lastAdd) > beamskip || frsm_dist(&points[i], &lastAddPoint) > spatialDecimationThresh || frsm_dist(&points[i],
        &origin) > (aveRange + 1.8 * stdDevRange)) {
      lastAdd = i;
      lastAddPoint = points[i];
      points[numDecPoints] = points[i]; // ok since i >= numDecPoints
      numDecPoints++;
    }
  }
  return numDecPoints;

}


void LidarOdom::doOdometry(std::vector<float> x, std::vector<float> y, int npoints, int64_t utime){
  doOdometry(x, y, npoints, utime, NULL);
}

void LidarOdom::doOdometry(std::vector<float> x, std::vector<float> y, int npoints, int64_t utime, ScanTransform* prior){
    double maxRange = 39.0;
    int beamSkip = 3;

    //Project ranges into points, and decimate points so we don't have too many
    frsmPoint * points = (frsmPoint *) calloc(npoints, sizeof(frsmPoint));
    int numValidPoints = projectPointsAndDecimate(beamSkip,
            cfg_.spatialDecimationThresh, x, y, npoints, points, maxRange);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n",
                numValidPoints);
        free(points);
        return;
    }

    ScanTransform r;

    r = sm_->matchSuccessive(points, numValidPoints,
            cfg_.laserType, utime, false, prior);
                                      //utime is ONLY used to tag the scans that get added to the map, doesn't actually matter

    Eigen::Isometry3d r_Iso = getScanTransformAsIsometry3d(r);

    prevOdom_ = currOdom_;
    currOdom_ = r_Iso;
    prevUtime_ = currUtime_;
    currUtime_ = utime;

    //Print current position periodically
    static double lastPrintTime = 0;
    if (frsm_get_time() - lastPrintTime > 2.0) {
        lastPrintTime = frsm_get_time();
        fprintf(stderr,
                "x=%+7.3f y=%+7.3f t=%+7.3f\t score=%f hits=%.2f sx=%.2f sxy=%.2f sy=%.2f st=%.2f, numValid=%d\n",
                r.x, r.y, r.theta, r.score, (double) r.hits / (double) numValidPoints, r.sigma[0], r.sigma[1],
                r.sigma[4], r.sigma[8], numValidPoints);
    }

    //Do drawing periodically
    // This was disabled when moving to the frsm library. it would be easily re-added on a fork
    //Do drawing periodically!
    if (cfg_.doDrawing && frsm_get_time() - lastDrawTime_ > .2) {
        lastDrawTime_ = frsm_get_time();
        //frsm_tictoc("drawing");
        draw(points, numValidPoints, &r);
        //frsm_tictoc("drawing");
    }

    //cleanup
    free(points);
}




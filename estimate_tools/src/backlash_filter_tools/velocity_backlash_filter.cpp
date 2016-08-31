//
// Created by manuelli on 8/31/16.
//

#include "velocity_backlash_filter.h"
#include <algorithm>

namespace EstimateTools{

  VelocityBacklashFilter::VelocityBacklashFilter(const double & slopTime, bool verbose):
      slopTime_(slopTime), init_(false), verbose_(verbose){};


  double VelocityBacklashFilter::processSample(const double & t, const double & v) {

    // initialize the filter
    if (!init_){
      tPrev_ = t;
      vPrev_ = v;
      init_ = true;
      timeSinceSloppy_ = 0;
      if (v > 0){
        filterState_ = BacklashFilterState::FORWARD_OK;
      }
      else {
        filterState_ = BacklashFilterState::BACKWARD_OK;
      }
      return v;
    }

    double dt = t - tPrev_;
    timeSinceSloppy_ += dt;

    bool sloppy = false;


    // implement filter logic
    switch(filterState_) {
      case FORWARD_OK:
        if (v < 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = BACKWARD_JUST_CROSSED;
        }
        break;

      case BACKWARD_OK:
        if (v > 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = FORWARD_JUST_CROSSED;
        }
        break;

      case FORWARD_JUST_CROSSED:
        if (v < 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = BACKWARD_SLOP;
        }
        else if (timeSinceSloppy_ > slopTime_){
          filterState_ = FORWARD_OK;
        }
        break;

      case BACKWARD_JUST_CROSSED:
        if (v > 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = FORWARD_SLOP;
        } else if (timeSinceSloppy_ > slopTime_){
          filterState_ = BACKWARD_OK;
        }
        break;

      case FORWARD_SLOP:
        if (v < 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = BACKWARD_SLOP;
        }
        else if (timeSinceSloppy_ > slopTime_){
          filterState_ = FORWARD_OK;
        }
        break;

      case BACKWARD_SLOP:
        if (v > 0.0){
          timeSinceSloppy_ = 0;
          filterState_ = FORWARD_SLOP;
        } else if (timeSinceSloppy_ > slopTime_){
          filterState_ = BACKWARD_OK;
        }
        break;
    }



    double percent = 1.0;

    // only apply the correction if we are in one of the sloppy states
    if (filterState_ == FORWARD_SLOP || filterState_ == BACKWARD_SLOP){
      percent = timeSinceSloppy_/slopTime_;
      percent = std::min(1.0, percent);
      percent = std::max(0.0, percent);
    }

    double vFiltered = percent*vPrev_;

    // book keeping
    tPrev_ = t;
    vPrev_ = v;

    return vFiltered; // return the filtered velocity
  }


  VelocityBacklashAlphaFilter::VelocityBacklashAlphaFilter(const double &breakFrequencyInHz, const double &slopTime):
  velocityBacklashFilter_(slopTime), singleAlphaFilter_(breakFrequencyInHz){}

  double VelocityBacklashAlphaFilter::processSample(const double &t, const double &v) {
    double vBacklashFiltered = velocityBacklashFilter_.processSample(t, v);
    double vAlphaFiltered = singleAlphaFilter_.processSample(t, vBacklashFiltered);

    return vAlphaFiltered;
  }
}

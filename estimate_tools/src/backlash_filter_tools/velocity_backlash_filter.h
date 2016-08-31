//
// Created by manuelli on 8/31/16.
//

#ifndef SOFTWARE_VELOCITY_BACKLASH_FILTER_H
#define SOFTWARE_VELOCITY_BACKLASH_FILTER_H

#include <estimate_tools/single_alpha_filter.hpp>
#include <estimate_tools/SimpleFilter.h>

namespace EstimateTools {


  enum BacklashFilterState{
    FORWARD_OK,
    BACKWARD_OK,
    FORWARD_JUST_CROSSED,
    BACKWARD_JUST_CROSSED,
    FORWARD_SLOP,
    BACKWARD_SLOP,
  };

  class VelocityBacklashFilter: public SimpleFilter{

  public:
    VelocityBacklashFilter(const double& slopTime, bool verbose = false);
    ~VelocityBacklashFilter(){}

    double processSample(const double & t, const double & v);

  private:
    const double & slopTime_;
    double tPrev_;
    double vPrev_;
    double timeSinceSloppy_;
    BacklashFilterState filterState_;

    bool init_;
    bool verbose_;
  };

  class VelocityBacklashAlphaFilter: public SimpleFilter{
  public:
    VelocityBacklashAlphaFilter(const double & breakFrequencyInHz, const double & slopTime = 0.05);
    ~VelocityBacklashAlphaFilter(){};

    double processSample(const double & t, const double & v);

  private:
    VelocityBacklashFilter velocityBacklashFilter_;
    EstimateTools::SingleAlphaFilter singleAlphaFilter_;
  };
}

#endif //SOFTWARE_VELOCITY_BACKLASH_FILTER_H

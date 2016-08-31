//
// Created by manuelli on 8/31/16.
//

#ifndef SOFTWARE_SIMPLEFILTER_H
#define SOFTWARE_SIMPLEFILTER_H

namespace EstimateTools{
  class SimpleFilter{
  public:
    virtual double processSample(const double & t, const double & x) = 0;
//    virtual ~SimpleFilter(){};
  };
}

#endif //SOFTWARE_SIMPLEFILTER_H

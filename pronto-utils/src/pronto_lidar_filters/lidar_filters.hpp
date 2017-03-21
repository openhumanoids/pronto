#ifndef PRONTO_LIDAR_FILTERS_HPP_
#define PRONTO_LIDAR_FILTERS_HPP_

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <math.h>

class pronto_lidar_filters {

public:
  pronto_lidar_filters(float minLimit, float maxLimit, float minRange, float maxRange);
  ~pronto_lidar_filters();

  void removeOuterPoints(const bot_core::planar_lidar_t *msg, bot_core::planar_lidar_t &msg_out);
  void removeOuterPoints(const bot_core::planar_lidar_t msg, bot_core::planar_lidar_t &msg_out);
  bot_core::planar_lidar_t removeOuterPoints(const bot_core::planar_lidar_t *msg);
  bot_core::planar_lidar_t removeRangePoints(const bot_core::planar_lidar_t *msg);

private:
  float minLimit_;
  float maxLimit_;
  float minRange_;
  float maxRange_;
};


#endif
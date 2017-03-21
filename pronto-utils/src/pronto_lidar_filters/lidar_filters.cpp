#include "lidar_filters.hpp"

/**
 * Constructor sets the limits for removing graound and range points
 */
pronto_lidar_filters::pronto_lidar_filters(float minLimit_, float maxLimit_, float minRange_, float maxRange_): minLimit_(minLimit_), maxLimit_(maxLimit_), minRange_(minRange_), maxRange_(maxRange_) {
}

/**
 * Cut the first and last X points from a received message
 * Configuration for X is defined by minLimit_ and maxLimit_
 *
 * @param msg     incomming planar_lidar_t message
 * @param msg_out filtered planar_lidar_t message
 */
void pronto_lidar_filters::removeOuterPoints(const bot_core::planar_lidar_t *msg, bot_core::planar_lidar_t & msg_out) {

  msg_out.utime = msg->utime;

  msg_out.radstep = msg->radstep;

  // difference between the wanted lower angle and the one in the message
  float dlr = fabs(msg->rad0 - minLimit_); // difference in radians

  int lb = round(dlr / msg->radstep);

  // (final angle of the sensor) - maxLimit
  float dur = (msg->rad0 + msg->nranges*msg->radstep) - maxLimit_;

  // array upper bound index
  int ub = msg->nranges - round(dur/msg->radstep);

  msg_out.nranges = ub-lb; // total ranges
  msg_out.nintensities = ub-lb; // total intensities

  msg_out.rad0 = msg->rad0 + lb*msg->radstep; // calculate the new starting angle

  for (int i=lb;i<ub; ++i) {
    msg_out.ranges.push_back(msg->ranges[i]);
    msg_out.intensities.push_back(msg->intensities[i]);
  }

}

/**
 * Cut the first and last X points from a received message
 * Configuration for X is defined by minLimit_ and maxLimit_
 *
 * @param msg     incomming planar_lidar_t message
 * @param msg_out filtered planar_lidar_t message
 */
void pronto_lidar_filters::removeOuterPoints(const bot_core::planar_lidar_t msg, bot_core::planar_lidar_t & msg_out) {

  msg_out.utime = msg.utime;

  msg_out.radstep = msg.radstep;

  // difference between the wanted lower angle and the one in the message
  float dlr = fabs(msg.rad0 - minLimit_); // difference in radians

  int lb = round(dlr / msg.radstep);

  // (final angle of the sensor) - maxLimit
  float dur = (msg.rad0 + msg.nranges*msg.radstep) - maxLimit_;

  // array upper bound index
  int ub = msg.nranges - round(dur/msg.radstep);

  msg_out.nranges = ub-lb; // total ranges
  msg_out.nintensities = ub-lb; // total intensities

  msg_out.rad0 = msg.rad0 + lb*msg.radstep; // calculate the new starting angle

  for (int i=lb;i<ub; ++i) {
    msg_out.ranges.push_back(msg.ranges[i]);
    msg_out.intensities.push_back(msg.intensities[i]);
  }

}

/**
 * Cut the first and last X points from a received message
 * Configuration for X is defined by minLimit_ and maxLimit_
 *
 * @param  msg incomming message
 * @return     filtered message
 */
bot_core::planar_lidar_t pronto_lidar_filters::removeOuterPoints(const bot_core::planar_lidar_t *msg) {

  bot_core::planar_lidar_t msg_out;
  msg_out.utime = msg->utime;

  msg_out.radstep = msg->radstep;

  // difference between the wanted lower angle and the one in the message
  float dlr = fabs(msg->rad0 - minLimit_); // difference in radians

  int lb = round(dlr / msg->radstep);

  // (final angle of the sensor) - maxLimit
  float dur = (msg->rad0 + msg->nranges*msg->radstep) - maxLimit_;

  // array upper bound index
  int ub = msg->nranges - round(dur/msg->radstep);

  msg_out.nranges = ub-lb; // total ranges
  msg_out.nintensities = ub-lb; // total intensities

  msg_out.rad0 = msg->rad0 + lb*msg->radstep; // calculate the new starting angle

  for (int i=lb;i<ub; ++i) {
    msg_out.ranges.push_back(msg->ranges[i]);
    msg_out.intensities.push_back(msg->intensities[i]);
  }

  return msg_out;

}

/**
 * Removes points in a bot_core::planar_lidar_t message that are further away than certain range and closer than certain range
 * @param  msg incoming message
 * @return     filtered message
 */
bot_core::planar_lidar_t pronto_lidar_filters::removeRangePoints(const bot_core::planar_lidar_t *msg) {

  bot_core::planar_lidar_t msg_out;
  msg_out.utime = msg->utime;

  msg_out.radstep = msg->radstep;

  msg_out.rad0 = msg->rad0; // similar starting angle

  for (size_t i=0;i<msg->ranges.size(); ++i) {
    if(msg->ranges[i] < maxRange_ && msg->ranges[i] > minRange_) {
      msg_out.ranges.push_back(msg->ranges[i]);
      msg_out.intensities.push_back(msg->intensities[i]);
    }
  }

  msg_out.nranges = msg_out.ranges.size();
  msg_out.nintensities = msg_out.intensities.size();

  return msg_out;

}
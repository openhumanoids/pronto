// Very Basic Complementary Filter
// Tested to be very similar to quaterion output from Microstrain GX3-25 (in hand)
// as well as HyQ's KVH sensor
//
// 1. Assumes that input sensor mesurements have been transformed into:
//    x forward +, y left +, z up +
//
// 2. Can only work when incoming measurements are in first quadrant
//    i.e. as described above
//
// TODO: Support 4-quadrant support by detecting the current quadrant and switching

#include "complementary.hpp"

#include <iostream>

Complementary::Complementary(double dt_):dt_(dt_)
{
  pitch_ = 0;
  roll_ = 0;
  yaw_ = 0;

  alpha_ = 0.02;
}

Complementary::~Complementary() {
}

void Complementary::update(Eigen::Vector3d accData, Eigen::Vector3d gyrData)
{
  roll_ += (gyrData[0]) * dt_; // Angle around the X-axis
  pitch_  += (gyrData[1]) * dt_;
  yaw_ += (gyrData[2]) * dt_;

  // Turning around the X axis results in a vector on the Y-axis
  rollAcc =  atan2f((float)accData[1], (float)accData[2]) ;//* 180.0 / M_PI ;
  roll_ = roll_ * (1-alpha_) + rollAcc * alpha_;

  // Turning around the Y axis results in a vector on the X-axis
  pitchAcc =  -atan2f((float)accData[0], (float)accData[2]) ;//* 180.0 / M_PI ;
  pitch_ = pitch_ * (1-alpha_) + pitchAcc * alpha_;
}


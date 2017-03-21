#ifndef __complementary_hpp__
#define __complementary_hpp__

#include <Eigen/Dense>
#include <Eigen/StdVector>

class Complementary {
public:
  Complementary(double dt_);
  virtual ~Complementary();

  void update(Eigen::Vector3d accData, Eigen::Vector3d gyrData);

  void setDT(double dt_in){ dt_ = dt_in; }
  void setAlpha(double alpha_in){ alpha_ = alpha_in; }

  double getPitch(){ return pitch_; }
  double getRoll(){ return roll_; }
  double getYaw(){ return yaw_; }

  double pitchAcc;
  double rollAcc;

private:
  Complementary (const Complementary& other);
  Complementary& operator=(const Complementary& other);


  double pitch_;
  double roll_;
  double yaw_;

  double alpha_;

  double dt_;
};

#endif


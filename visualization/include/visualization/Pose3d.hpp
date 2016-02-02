/** @class isam::Pose3d
 *
 * Conventions:
 *
 * Right-handed coordinate system (NED: north-east-down)
 * X forward (along default motion of robot)
 * Y right
 * Z down
 *
 * Rotations are represented using standard Euler angles
 * yaw
 * pitch
 * roll
 *
 * Note that Euler angles transform objects from the global into the local frame
 * of the vehicle: First yaw rotates around Z (changing X and Y axes to X' and Y'),
 * then pitch around the new Y' axis, and finally roll around the new X' axis.
 *
 * In contrast, the returned rotation and transformation matrices are defined
 * in the opposite direction:
 * wTo transforms a point from the local (second) system to the global (first) system
 *
 */

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "util.hpp"
#include "Rot3d.hpp"
//#include "Pose2d.h"
#include "Point3d.hpp"
//#include "Point3dh.h"
//#include "Point2d.h"

namespace coll {

typedef Eigen::Matrix< double, 6, 1> Vector6d;

class Pose3d {
  friend std::ostream& operator<<(std::ostream& out, const Pose3d& p) {
    p.write(out);
    return out;
  }

  Point3d _t;
  Rot3d _rot;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int dim = 6;
  static const char* name() {
    return "Pose3d";
  }

  Pose3d() : _t(0,0,0), _rot(0,0,0) {}

  Pose3d(double x, double y, double z, double yaw, double pitch, double roll) : _t(x, y, z), _rot(yaw, pitch, roll) {}

  Pose3d(const Eigen::MatrixXd& m) {
    if (m.rows()==6 && m.cols()==1) {
      _t = Point3d(m(0), m(1), m(2));
      _rot = Rot3d(m(3), m(4), m(5));
    } else if (m.rows()==4 && m.cols()==4) {
      // Convert a homogeneous 4x4 transformation matrix to a Pose3.
      Eigen::Matrix4d wTo = m / m(3,3); // enforce T(3,3)=1
      Eigen::Vector3d t = wTo.col(3).head(3);
      Eigen::Matrix3d wRo = wTo.topLeftCorner(3,3);
      _t = Point3d(t(0), t(1), t(2));
      _rot = Rot3d(wRo);
    } else {
      std::cout << "Pose3d constructor called with matrix of wrong dimension\n";
      exit(-1);
    }
  }

  explicit Pose3d(const Eigen::Isometry3d & T) {
    Eigen::Vector3d t(T.translation());
    _t = Point3d(t(0), t(1), t(2));
    _rot = Rot3d(T.rotation());
  }

  Pose3d(const Point3d& t, const Rot3d& rot) : _t(t), _rot(rot) {}

  double x() const {return _t.x();}
  double y() const {return _t.y();}
  double z() const {return _t.z();}
  double yaw()   const {return _rot.yaw();}
  double pitch() const {return _rot.pitch();}
  double roll()  const {return _rot.roll();}

  Point3d trans() const {return _t;}
  Rot3d rot() const {return _rot;}

  void set_x(double x) {_t.set_x(x);}
  void set_y(double y) {_t.set_y(y);}
  void set_z(double z) {_t.set_z(z);}
  void set_yaw  (double yaw)   {_rot.set_yaw(yaw);}
  void set_pitch(double pitch) {_rot.set_pitch(pitch);}
  void set_roll (double roll)  {_rot.set_roll(roll);}

  Pose3d exmap(const Vector6d& delta) const {
    Pose3d res = *this;
    res._t   = res._t.exmap(delta.head(3));
    res._rot = res._rot.exmap(delta.tail(3));
    return res;
  }

  Vector6d vector() const {
    double Y, P, R;
    // cheaper to recover ypr at once
    _rot.ypr(Y, P, R);
    Vector6d tmp;
    tmp << x(), y(), z(), Y, P, R;
    return tmp;
  }

  void set(double x, double y, double z, double yaw, double pitch, double roll) {
    _t = Point3d(x, y, z);
    _rot = Rot3d(yaw, pitch, roll);
  }

  void set(const Vector6d& v) {
    _t = Point3d(v(0), v(1), v(2));
    _rot = Rot3d(standardRad(v(3)), standardRad(v(4)), standardRad(v(5)));
  }

  void of_point3d(const Point3d& p) {
    set(p.x(), p.y(), p.z(), 0., 0., 0.);
  }

  void write(std::ostream &out) const {
    out << "(" << x() << ", " << y() << ", " << z() << "; "
        << yaw() << ", " << pitch() << ", " << roll() << ")";
  }

  /**
   * Convert Pose3 to homogeneous 4x4 transformation matrix.
   * The returned matrix is the object coordinate frame in the world
   * coordinate frame. In other words it transforms a point in the object
   * frame to the world frame.
   *
   * @return wTo
   */
  Eigen::Matrix4d wTo() const {
    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = _rot.wRo();
    T.col(3).head(3) << x(), y(), z();
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }

  /**
   * Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
   * The returned matrix is the world coordinate frame in the object
   * coordinate frame. In other words it transforms a point in the world
   * frame to the object frame.
   *
   * @return oTw
   */
  Eigen::Matrix4d oTw() const {
    Eigen::Matrix3d oRw = _rot.wRo().transpose();
    Eigen::Vector3d t(x(), y(), z());
    Eigen::Vector3d C = - oRw * t;
    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = oRw;
    T.col(3).head(3) = C;
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }

  /**
   * Calculate new pose b composed from this pose (a) and the odometry d.
   * Follows notation of Lu&Milios 1997.
   * \f$ b = a \oplus d \f$
   * @param d Pose difference to add.
   * @return d transformed from being local in this frame (a) to the global frame.
   */
  Pose3d oplus(const Pose3d& d) const {
    return Pose3d(wTo() * d.wTo());
  }

  /**
   * Odometry d from b to this pose (a). Follows notation of
   * Lu&Milios 1997.
   * \f$ d = a \ominus b \f$
   * @param b Base frame.
   * @return Global this (a) expressed in base frame b.
   */
  Pose3d ominus(const Pose3d& b) const {
    return Pose3d(b.oTw() * wTo());
  }

};

}

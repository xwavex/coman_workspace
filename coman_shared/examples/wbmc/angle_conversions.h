/**
 * @file angle_conversion.h
 * @brief Converting between Euler angles, Quaternions and rotation matrices.
 * @author Michael Kaess
 * @version $Id: $
 *
 */

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

Eigen::Matrix3d euler_to_wRo(double yaw, double pitch, double roll) {
  double c__ = cos(yaw);
  double _c_ = cos(pitch);
  double __c = cos(roll);
  double s__ = sin(yaw);
  double _s_ = sin(pitch);
  double __s = sin(roll);
  double cc_ = c__ * _c_;
  double cs_ = c__ * _s_;
  double sc_ = s__ * _c_;
  double ss_ = s__ * _s_;
  double c_c = c__ * __c;
  double c_s = c__ * __s;
  double s_c = s__ * __c;
  double s_s = s__ * __s;
  double _cc = _c_ * __c;
  double _cs = _c_ * __s;
  double csc = cs_ * __c;
  double css = cs_ * __s;
  double ssc = ss_ * __c;
  double sss = ss_ * __s;
  Eigen::Matrix3d wRo;
  wRo <<
    cc_  , css-s_c,  csc+s_s,
    sc_  , sss+c_c,  ssc-c_s,
    -_s_  ,     _cs,      _cc;
  return wRo;
}

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

Eigen::Quaterniond wRo_to_quat(const Eigen::Matrix3d& wRo) {
  return Eigen::Quaterniond(wRo);
}

Eigen::Matrix3d quat_to_wRo(const Eigen::Quaterniond& quat) {
  return Eigen::Matrix3d(quat);
}

Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

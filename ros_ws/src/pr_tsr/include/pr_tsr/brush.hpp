#ifndef PR_TSR_BRUSH_HPP
#define PR_TSR_BRUSH_HPP

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace pr_tsr {
inline aikido::constraint::dart::TSR getDefaultBrushTSR()
{
  using Eigen::Isometry3d;
  aikido::constraint::dart::TSR tsr;

  double brush_height = 0.039;
  double brush_length = 0.180;
  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d orientation;
  // clang-format off
  orientation << 
      0, 0, 1,
      1, 0, 0,
      0, 1, 0;
  // clang-format on
  tsr.mT0_w.linear() = orientation;
  tsr.mT0_w.translation() = Eigen::Vector3d(0, -brush_length/2.0, 0);

  // Transform between end effector and w
  Eigen::Isometry3d Tw_e = Eigen::Isometry3d::Identity(); 
  Tw_e.translation() = Eigen::Vector3d(brush_length ,0, 0);
  Eigen::Matrix3d rot;
  // clang-format off
  rot << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  // clang-format on
  Tw_e.linear() = rot;
  tsr.mTw_e = Tw_e;

  // Rotation around object
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  double vertical_tolerance = 0.02;
  Bw(2, 0) = -vertical_tolerance;
  Bw(2, 1) = vertical_tolerance;
  Bw(0, 0) = -0.1; 
  Bw(0, 1) = 0.1;
  tsr.mBw = Bw;

  return tsr;
}
}
#endif // PR_TSR_BRUSH_HPP

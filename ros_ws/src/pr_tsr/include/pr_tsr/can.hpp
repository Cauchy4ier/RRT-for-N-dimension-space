#ifndef PR_TSR_CAN_HPP
#define PR_TSR_CAN_HPP

#include <Eigen/Core>
#include <aikido/constraint/dart/TSR.hpp>
#include <dart/math/math.hpp>

namespace pr_tsr {
inline aikido::constraint::dart::TSR getDefaultCanTSR()
{
  using Eigen::Isometry3d;
  aikido::constraint::dart::TSR tsr;

  // Transform w.r.t root
  tsr.mT0_w = Eigen::Isometry3d::Identity();


 
  Isometry3d Tw_e = Eigen::Isometry3d::Identity();
  
  tsr.mTw_e = Tw_e;

  // Rotation around object
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  tsr.mBw = Bw;

  return tsr;
}
}
#endif // PR_TSR_CAN_HPP

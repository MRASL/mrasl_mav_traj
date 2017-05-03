/*******************************************************************************
 * This file is part of mrasl_mav_traj.
 *
 * Copyright 2017 Mobile Robotics and autonomous Systems Laboratory (MRASL),
 * Polytechnique Montreal. All rights reserved.
 *
 * If you use this code, please cite the respective publications.
 *
 * mrasl_mav_traj is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mrasl_mav_traj is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mrasl_mav_traj.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

/**
 * @file TrajectorySegment.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#ifndef TRAJECTORYSEGMENT_HPP_H
#define TRAJECTORYSEGMENT_HPP_H
#include <Eigen/Core>

#include "mrasl_mav_traj/TrajectoryMath.hpp"

using namespace Eigen;

namespace mrasl {
class TrajectorySegment {
public:

  /**
   * Non dimensional trajectory segment. Defined on interval 0 to 1
   * @param polynomial
   */
  TrajectorySegment(VectorXd polynomial[3]);

  /**
   *
   * @param t0    Start time used in the opt problem
   * @param tend  End time used in the opt problme
   * @param alpha Time scale we suppose t0 and tend were NOT already multiplied
   * by alpha
   * @param polynomial
   */
  TrajectorySegment(double   t0,
                    double   tend,
                    double   alpha,
                    VectorXd polynomial[3]);
  TrajectorySegment(VectorXd x,
                    VectorXd y,
                    VectorXd z);

  /**
   *
   * @param t0    Start time
   * @param tend  End time
   * @param alpha Timescaling factor
   * @param x     Polynomial in x
   * @param y     Polynomial in y
   * @param z     Polynomial in z
   */
  TrajectorySegment(double   t0,
                    double   tend,
                    double   alpha,
                    VectorXd x,
                    VectorXd y,
                    VectorXd z);

  std::vector<Vector3d>getPositionTrajectory();
  std::vector<Vector3d>getVelocityTrajectory();
  std::vector<Vector3d>getAccelerationTrajectory();
  std::vector<Vector3d>getTraj(Derivative der);

  std::vector<Vector3d>discretize(double dt);

private:

  double t0_, tend_;
  double alpha_;                            // time scale

  VectorXd polynomial_[State::STATE_COUNT]; // Non-dimensionalized trajectory
                                            // segment
  std::vector<Vector3d> trajectory_[Derivative::DER_ACCELERATION + 1];
};
}

#endif // RAJECTORYSEGMENT_HPP_H

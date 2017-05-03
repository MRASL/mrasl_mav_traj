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
 * @file TrajectoryConstraint.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/
#ifndef TRAJECTORYCONSTRAINT_H
#define TRAJECTORYCONSTRAINT_H

#include <Eigen/Core>
#include "TrajectoryMath.hpp"

using namespace Eigen;

namespace mrasl {
class TrajectoryConstraint {
public:

  /**
   * By default all constraints are set to unconstrained (i.e.
   * std::numeric_limits<double>::max())
   */
  TrajectoryConstraint(double time);
  TrajectoryConstraint(double   time,
                       Vector3d position);
  TrajectoryConstraint(double   time,
                       Vector3d position,
                       Vector3d velocity);
  TrajectoryConstraint(double   time,
                       Vector3d position,
                       Vector3d velocity,
                       Vector3d acceleration);
  TrajectoryConstraint(double   time,
                       Vector3d position,
                       Vector3d velocity,
                       Vector3d acceleration,
                       Vector3d jerk);
  TrajectoryConstraint(double   time,
                       Vector3d position,
                       Vector3d velocity,
                       Vector3d acceleration,
                       Vector3d jerk,
                       Vector3d snap);

  Vector3d getPosition() const;
  Vector3d getVelocity() const;
  Vector3d getAcceleration() const;
  Vector3d getJerk() const;
  Vector3d getSnap() const;
  Vector3d getConstraint(int derivative) const;
  double   getPosition(int dim) const;
  double   getVelocity(int dim) const;
  double   getAcceleration(int dim) const;
  double   getJerk(int dim) const;
  double   getSnap(int dim) const;
  double   getTime() const;

  /**
   * Counts the number of constrained derivatives in a
   * certain dimension.
   * @param dim Dimension to check
   * @return
   */
  int      getConstraintCount(int dim) const;

  void     setPosition(const Vector3d pos);
  void     setVelocity(const Vector3d vel);
  void     setAcceleration(const Vector3d acc);
  void     setJerk(const Vector3d jerk);
  void     setSnap(const Vector3d snap);
  void     setConstraint(const int      derivative,
                         const Vector3d constraint);
  void     setTime(const double time);

  /**
   * Checks if the derivative of a certain dimension is constrained
   * @param derivative
   * @param dimension
   * @return
   */
  bool     isConstrained(int derivative,
                         int dimension) const;

private:

  Vector3d constraints_[Derivative::DER_COUNT];
  double   time_;
};
}

#endif // TRAJECTORYCONSTRAINT_H

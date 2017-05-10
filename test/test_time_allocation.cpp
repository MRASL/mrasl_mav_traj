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
 * @file test_trajectory_generation.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <gtest/gtest.h>
#include <Eigen/Core>
#include "mrasl_mav_traj/TrajectoryGenerator.hpp"
#include "mrasl_mav_traj/TimeAllocator.hpp"
#include "mrasl_mav_traj/nlp_solvers/NLPalg.hpp"
#include "mrasl_mav_traj/nlp_solvers/NLPnlopt.hpp"

using namespace mrasl;
using namespace Eigen;

TEST(time_allocation, tesl_alg) {
  Vector3d wp0, wp1, wp2, wp3;
  wp0 << 0, 0, 0;
  wp1 << 1, 0, 0;
  wp2 << 1, 2, 0;
  wp3 << 0, 2, 0;

  TrajectoryGenerator *tg = new TrajectoryGenerator();
  tg->addConstraint(
      TrajectoryConstraint(0, wp0, Vector3d::Zero(), Vector3d::Zero(),
                           Vector3d::Zero(), Vector3d::Zero()));
  tg->addConstraint(TrajectoryConstraint(0.5, wp1));
  tg->addConstraint(TrajectoryConstraint(2.5, wp2));
  tg->addConstraint(
      TrajectoryConstraint(3, wp3, Vector3d::Zero(), Vector3d::Zero(),
                           Vector3d::Zero(), Vector3d::Zero()));

  TimeAllocator::generator_solver_pair_t gsp;
  gsp.tg = tg;
  gsp.solver = TrajectoryGenerator::Solver::LINEAR_SOLVE;
  TimeAllocator *ta = new NLPalg(gsp);
  ta->optimize();
  VectorXd solution = tg->getArrivalTimes();
  double expected[] = {0.0,	0.975874415288207,	2.02418446511101,	3.0};
  EXPECT_DOUBLE_EQ(expected[0], solution(0));
  EXPECT_NEAR(expected[1], solution(1), 1e-2);  // We're not as good as quadprog
  EXPECT_NEAR(expected[2], solution(2), 1e-2);
  EXPECT_DOUBLE_EQ(expected[3], solution(3));  // Accept rounding errors
}

TEST(time_allocation, tesl_nlopt) {
  Vector3d wp0, wp1, wp2, wp3;
  wp0 << 0, 0, 0;
  wp1 << 1, 0, 0;
  wp2 << 1, 2, 0;
  wp3 << 0, 2, 0;

  TrajectoryGenerator *tg = new TrajectoryGenerator();
  tg->addConstraint(
      TrajectoryConstraint(0, wp0, Vector3d::Zero(), Vector3d::Zero(),
                           Vector3d::Zero(), Vector3d::Zero()));
  tg->addConstraint(TrajectoryConstraint(0.5, wp1));
  tg->addConstraint(TrajectoryConstraint(2.5, wp2));
  tg->addConstraint(
      TrajectoryConstraint(3, wp3, Vector3d::Zero(), Vector3d::Zero(),
                           Vector3d::Zero(), Vector3d::Zero()));

  TimeAllocator::generator_solver_pair_t gsp;
  gsp.tg = tg;
  gsp.solver = TrajectoryGenerator::Solver::LINEAR_SOLVE;
  TimeAllocator *ta = new NLPnlopt(gsp);
  ta->optimize();
  VectorXd solution = tg->getArrivalTimes();
  double expected[] = {0.0,	0.975874415288207,	2.02418446511101,	3.0};
  EXPECT_DOUBLE_EQ(expected[0], solution(0));
  EXPECT_NEAR(expected[1], solution(1), 1e-2);  // We're not as good as quadprog
  EXPECT_NEAR(expected[2], solution(2), 1e-2);
  EXPECT_DOUBLE_EQ(expected[3], solution(3));  // Accept rounding errors
}
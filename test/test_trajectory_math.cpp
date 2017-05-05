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
 * @file test_trajectory_math.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <mrasl_mav_traj/TrajectoryMath.hpp>

using namespace mrasl;
using namespace Eigen;

TEST(traj_math, polyval0) {
  Vector3d v;

  v << 3, 2, 1;
  double answer = polyval(v, 5.0);
  ASSERT_TRUE(answer == 86);
  answer = polyval(v, 7);
  ASSERT_TRUE(answer == 162);
  answer = polyval(v, 9);
  ASSERT_TRUE(answer == 262);
}

TEST(traj_math, polyval1) {
  Vector3d v, x;

  v << 3, 2, 1;
  x << 5, 7, 9;
  Vector3d ans = polyval(v, x);
  ASSERT_TRUE(ans(0) == 86);
  ASSERT_TRUE(ans(1) == 162);
  ASSERT_TRUE(ans(2) == 262);
}

TEST(traj_math, polyder0) {
  VectorXd v(7);

  v << 1, 1, 1, 1, 1, 1, 1;
  VectorXd vdev(6);
  vdev << 6, 5, 4, 3, 2, 1;
  auto answer = polyder(v);

  ASSERT_TRUE(answer.isApprox(vdev));

  answer = polyder(vdev);
  VectorXd vans(5);
  vans << 30, 20, 12, 6, 2;
  ASSERT_TRUE(answer.isApprox(vans));
}

TEST(traj_math, rot90) {
  MatrixXd I(3, 3);

  I << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  auto answer = rot90(I);
  MatrixXd expected(3, 3);
  expected << 3, 6, 9, 2, 5, 8, 1, 4, 7;
  ASSERT_TRUE(answer.isApprox(expected));

  answer = rot90(I, 2);
  expected << 9, 8, 7, 6, 5, 4, 3, 2, 1;
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, getCoefficients) {
  MatrixXd expected(5, 7);

  expected <<     1, 1, 1, 1, 1, 1, 1
  , 6, 5, 4, 3, 2, 1, 0
  , 30, 20, 12, 6, 2, 0, 0
  , 120, 60, 24, 6, 0, 0, 0
  , 360, 120, 24, 0, 0, 0, 0;

  auto answer = genCoefficientMatrix(6, 4);
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, rightpad) {
  VectorXd expected(4), in(2);

  expected << 1, 2, 0, 0;
  in << 1, 2;
  VectorXd answer = rightPadZeros(in, 4);
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, leftpad) {
  VectorXd expected(4), in(2);

  expected << 0, 0, 1, 2;
  in << 1, 2;
  VectorXd answer = leftPadZeros(in, 4);
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, scalarpowered) {
  VectorXd expected(3), powers(3);

  expected << 8, 4, 2;
  powers << 3, 2, 1;
  VectorXd answer = scalarPowered(2, powers);
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, eigenmat2buf) {
  MatrixXd in(2, 2);

  in << 1.0, 2.0,
  3.0, 4.0;
  double buf[4];
  eigenMat2buf(in, buf);

  double expected = 1.0;

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      ASSERT_TRUE(in(i, j) == expected);
      expected += 1.0;
    }
  }
}

TEST(traj_math, time2segment) {
  VectorXd arrival_times(4), segment_times(3);

  arrival_times << 0, 1, 2, 3;
  segment_times << 1, 1, 1;
  VectorXd answer = time2segment(arrival_times);
  ASSERT_TRUE(answer.isApprox(segment_times));
}

TEST(traj_math, segment2time) {
  VectorXd arrival_times(4), segment_times(3);

  arrival_times << 0, 1, 2, 3;
  segment_times << 1, 1, 1;
  VectorXd answer = segment2time(segment_times);
  ASSERT_TRUE(answer.isApprox(arrival_times));
}

TEST(traj_math, segmenttimerealloc) {
  VectorXd arrival_times(5), delta(4), expected(5);

  arrival_times << 0.0, 1.0, 2.0, 3.0, 4.0;
  delta << 0.25, 0.25, -0.25, -0.25;
  expected << 0.0, 1.25, 2.50, 3.25, 4.0;
  VectorXd answer = segtimeRealloc(arrival_times, delta);
  ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, matrixconcat) {
  MatrixXd A(2,3), B(3,3), K(5, 5);
  A << 1, 1, 1, 2, 2, 2;
  B.fill(3);
  K << B, A.transpose(), A, MatrixXd::Zero(2,2);

  MatrixXd expected(5,5);
  expected << 3, 3, 3, 1, 2,
              3, 3, 3, 1, 2,
              3, 3, 3, 1, 2,
              1, 1, 1, 0, 0,
              2, 2, 2, 0, 0;
  ASSERT_TRUE(K.isApprox(expected));
}

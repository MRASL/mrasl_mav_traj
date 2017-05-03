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
 * @file TrajectoryMath.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <cmath>
#include <Eigen/Core>
#include <iostream>
#include "mrasl_mav_traj/TrajectoryMath.hpp"

using namespace Eigen;

namespace mrasl {
double polyval(VectorXd coefficients, double x) {
  double sum = 0;

  // iterate on everything except the last element
  for (int i = 0; i < coefficients.rows() - 1; ++i) {
    long n = coefficients.rows() - 1;
    sum += coefficients(i) * std::pow(x, n - i);
  }
  sum += coefficients.tail(1) (0);
  return sum;
}

VectorXd polyval(VectorXd coefficients, VectorXd x) {
  VectorXd res = VectorXd::Zero(x.rows());

  for (int i = 0; i < x.rows(); ++i) {
    res(i) = polyval(coefficients, x(i));
  }
  return res;
}

VectorXd polyder(VectorXd coefficients) {
  // We don't check if the vector is 1d because that
  // doesn't exist with eigen.
  VectorXd derived       = VectorXd::Zero(coefficients.rows() - 1);
  long     highest_power = coefficients.rows() - 1;

  for (int i = 0; i < highest_power; ++i) {
    derived(i) = coefficients(i) * (highest_power - i);
  }
  return derived;
}

Eigen::MatrixXd rot90(Eigen::MatrixXd m) {
  MatrixXd r = m;

  for (int i = 0; i < m.rows(); ++i) {
    r.row(i) = m.col(m.cols() - 1 - i);
  }
  return r;
}

Eigen::MatrixXd rot90(Eigen::MatrixXd m, int k) {
  for (int i = 0; i < k; ++i) {
    m = rot90(m);
  }
  return m;
}

Eigen::VectorXd rightPadZeros(VectorXd vec, int s) {
  if (vec.size() >= s) {
    return vec;
  } else {
    VectorXd padded(s);
    padded << vec, VectorXd::Zero(s - vec.size());
    return padded;
  }
}

Eigen::VectorXd leftPadZeros(Eigen::VectorXd vec, int s) {
  if (vec.size() >= s) {
    return vec;
  } else {
    VectorXd padded(s);
    padded << VectorXd::Zero(s - vec.size()), vec;
    return padded;
  }
}

Eigen::MatrixXd genCoefficientMatrix(int n, int r) {
  int n_coeffs        = n + 1;
  int n_poly          = r + 1;
  MatrixXd coeffs     = MatrixXd::Zero(n_poly, n_coeffs);
  VectorXd polynomial = VectorXd::Ones(n_coeffs);

  for (int i = 0; i < n_poly; ++i) {
    coeffs.row(i) = rightPadZeros(polynomial, n_coeffs);
    polynomial    = polyder(polynomial);
  }
  return coeffs;
}

Eigen::VectorXd scalarPowered(double scalar, Eigen::VectorXd powers) {
  VectorXd res = VectorXd::Ones(powers.size());

  for (int i = 0; i < powers.size(); ++i) {
    res(i) = std::pow(scalar, powers(i));
  }
  return res;
}

void eigenMat2buf(Eigen::MatrixXd mat, double buf[]) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = mat;

  for (int i = 0; i < rm.size(); ++i) {
    buf[i] = *(rm.data() + i);
  }
}

Eigen::VectorXd time2segment(Eigen::VectorXd t) {
  return t.tail(t.size() - 1) - t.head(t.size() - 1);
}

Eigen::VectorXd segment2time(Eigen::VectorXd T) {
  VectorXd times(T.size() + 1);

    times(0) = 0;

  for (int i = 1; i < times.size(); ++i) {
    times(i) = times(i - 1) + T(i - 1);
  }
  return times;
}

Eigen::VectorXd segtimeRealloc(Eigen::VectorXd t, Eigen::VectorXd delta) {
  double   orig_last = t(t.size() - 1);
  VectorXd seg_times = time2segment(t);

  seg_times += delta;
  VectorXd reallocated_t = segment2time(seg_times);

  assert(t(t.size() - 1) == orig_last);
  return reallocated_t;
}
}

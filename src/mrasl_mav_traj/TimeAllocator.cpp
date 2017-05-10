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
 * @file TimeAllocator.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include "mrasl_mav_traj/TimeAllocator.hpp"

namespace mrasl {
  constexpr double TimeAllocator::h_;
  TimeAllocator::TimeAllocator(generator_solver_pair_t pair) : pair_(pair) {}

  Eigen::MatrixXd TimeAllocator::generateGi(const int m) {
    MatrixXd mat(m,m);
    mat.fill(-1.0/(m-1));
    for(int i = 0; i < m; ++i)
      mat(i,i) = 1;
    return mat;
  }
}
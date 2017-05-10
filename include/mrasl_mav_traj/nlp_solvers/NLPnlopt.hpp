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
 * @file NLPnlopt.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#ifndef MRASL_MAV_TRAJ_NLPNLOPT_HPP
#define MRASL_MAV_TRAJ_NLPNLOPT_HPP

#include "mrasl_mav_traj/TimeAllocator.hpp"

namespace mrasl {
  class NLPnlopt : public TimeAllocator {
  public:
    NLPnlopt(generator_solver_pair_t pair);

    bool optimize();

    static double nlopt_obj_func(const std::vector<double> &x,
                                 std::vector<double> &grad, void *f_data);

    static double nlopt_constr(const std::vector<double> &x,
                               std::vector<double> &grad, void *data);

  private:

  };
}

#endif //MRASL_MAV_TRAJ_NLPNLOPT_HPP

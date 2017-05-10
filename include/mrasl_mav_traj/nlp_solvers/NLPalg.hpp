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
 * @file NLPalg.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#ifndef MRASL_MAV_TRAJ_NLPALG_H
#define MRASL_MAV_TRAJ_NLPALG_H

#include <alglib/optimization.h>
#include "mrasl_mav_traj/TimeAllocator.hpp"

namespace mrasl {
    class NLPalg : public TimeAllocator {
    public:
        NLPalg(generator_solver_pair_t pair);

        bool optimize();

        static void timeAllocGrad(const alglib::real_1d_array &x, double &func,
                                  alglib::real_1d_array &grad, void *ptr);

    private:
        static alglib::real_1d_array eigen2alg(const Eigen::VectorXd vec);
        static alglib::real_2d_array eigen2alg(const Eigen::MatrixXd mat);
        static Eigen::VectorXd alg2eigen(const alglib::real_1d_array r);
        static int func_evals_;
    };
}

#endif //MRASL_MAV_TRAJ_NLPALG_H

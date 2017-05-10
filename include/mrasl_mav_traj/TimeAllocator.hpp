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
 * @file TimeAllocator.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#ifndef MRASL_MAV_TRAJ_TIMEALLOCATOR_H
#define MRASL_MAV_TRAJ_TIMEALLOCATOR_H

#include "mrasl_mav_traj/TrajectoryGenerator.hpp"

namespace mrasl {
    /**
     * Take waypoint arrival times be t_i where i [1, m] (m is the num of wps)
     * we have T_i = t_(i+1) - t_i the segment times instead of the arrival
     * times. Let f(T) be the solution of the QP problem being solved to get
     * the minimum snap trajectory we now consider the problem
     *
     * min  f(T)
     *  T
     * s.t. sum(T_i) = t_m      i = 1, ..., m
     *      T_i     >= 0        i = 1, ..., m
     *
     * The gradient is computed numerically where
     *      nabla f = ( f(T+ h*g_i) - f(T) ) / h
     * Where
     *      g_i =   1           :   at position i
     *              -1/(m-1)    :   otherwise
     */
    class TimeAllocator {
    public:
        typedef struct {
            TrajectoryGenerator* tg;
            TrajectoryGenerator::Solver solver;
            int iters;
        } generator_solver_pair_t;

        TimeAllocator(generator_solver_pair_t pair);

        /**
         * Optimize the time allocation of trajectory segments
         * @return
         */
        virtual bool optimize() = 0;

    protected:
        static constexpr double h_ = 0.001;
        generator_solver_pair_t pair_;

        /**
         * Generate the gradient directions
         * @param m Number of SEGMENTS i.e. n_segments = n_keyframes - 1
         * @return
         */
        static Eigen::MatrixXd generateGi(const int m);
    };
}

#endif //MRASL_MAV_TRAJ_TIMEALLOCATOR_H

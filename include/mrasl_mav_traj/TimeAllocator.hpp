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
    class TimeAllocator {
    public:
        typedef struct {
            TrajectoryGenerator* tg;
            TrajectoryGenerator::Solver solver;
            int iters;
        } generator_solver_pair_t;

        TimeAllocator(generator_solver_pair_t pair) : pair_(pair) {}

        /**
         * Optimize the time allocation of trajectory segments
         * @return
         */
        virtual bool optimize() = 0;

    protected:
        generator_solver_pair_t pair_;

        /**
         * Generate the gradient directions
         * @param m Number of SEGMENTS i.e. n_segments = n_keyframes - 1
         * @return
         */
        static Eigen::MatrixXd generateGi(const int m) {
            MatrixXd mat(m,m);
            mat.fill(-1.0/(m-1));
            for(int i = 0; i < m; ++i)
                mat(i,i) = 1;
            return mat;
        }
    };
}

#endif //MRASL_MAV_TRAJ_TIMEALLOCATOR_H

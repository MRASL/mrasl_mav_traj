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
 * @file QPProblem.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

 #include "mrasl_mav_traj/qp_solvers/QPProblem.hpp"

 namespace mrasl {
   QPProblem::QPProblem(const MatrixXd H, const VectorXd c, const MatrixXd A,
     const VectorXd b) : H_(H), c_(c), A_(A), b_(b) {

     }

     QPProblem::QPProblem(const MatrixXd H, const MatrixXd A, const VectorXd b)
     : H_(H), A_(A), b_(b) {
       c_ = VectorXd::Zero(H_.cols());
     }
}

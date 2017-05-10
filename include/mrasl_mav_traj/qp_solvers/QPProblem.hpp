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
 * @file QPProblem.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/
#ifndef QPPROBLEM_HPP
#define QPPROBLEM_HPP

#include <Eigen/Core>

using namespace Eigen;

namespace mrasl {
/*******************************************************************************
 *  QPProblem is a class that helps formulate QP problems with linear
 *  constraints of the form
 *  min  0.5 x' H x + c' x
 *   x
 *  s.t. Ax = b
 ******************************************************************************/
    class QPProblem {
    public:

        /**
         * Constructor
         * @param H Hessian
         * @param c Linear term
         * @param A Linear constraint matrix
         * @param b Linear constraint vector
         */
        QPProblem(const MatrixXd H, const VectorXd c, const MatrixXd A,
                  const VectorXd b);

        /**
         * Constructor where linear term is zero
         * @param H Hessian
         * @param A Linear constraint matrix
         * @param b Linear constraint vector
         */
        QPProblem(const MatrixXd H, const MatrixXd A, const VectorXd b);
        ~QPProblem(){};

        virtual bool solve() = 0;

        VectorXd getSolution(){ return solution_; }
    private:
        QPProblem();

    protected:
        MatrixXd H_, A_;
        VectorXd c_, b_;
        VectorXd solution_;
    };
}

#endif // QPPROBLEM_HPP

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
 * @file TrajectoryGenerator.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <alglib/optimization.h>
#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>

#include <mrasl_mav_traj/TrajectoryConstraint.hpp>
#include <iostream>
#include "mrasl_mav_traj/TrajectorySegment.hpp"
#include "mrasl_mav_traj/TrajectoryGenerator.hpp"

namespace mrasl {
    TrajectoryGenerator::TrajectoryGenerator() {
        problemBuilt_ = false;
        isDiscretized_ = false;
    }

    /***************************************************************************
     *
     *  Getters
     *
     **************************************************************************/

    std::vector<TrajectoryConstraint> TrajectoryGenerator::getConstraints(){
        return keyframes_;
    }

    unsigned long TrajectoryGenerator::getNumWaypoints() {
        return keyframes_.size();
    }

    unsigned long TrajectoryGenerator::getNumVars() {
        return n_coeffs_ * (getNumWaypoints()-1);
    }

    VectorXd TrajectoryGenerator::getArrivalTimes() const {
        VectorXd times(keyframes_.size());
        for(int i = 0; i < keyframes_.size(); ++i){
            times(i) = keyframes_[i].getTime();
        }
        return times;
    }

    MatrixXd TrajectoryGenerator::getCostMatrix(int dim) const {
        return H_[dim];
    }

    MatrixXd TrajectoryGenerator::getConstraintMatrix(int dim) const {
        return Aeq_[dim];
    }

    VectorXd TrajectoryGenerator::getConstraintVector(int dim) const {
        return beq_[dim];
    }

    VectorXd TrajectoryGenerator::getSolution(int dim) const {
        return solution_[dim];
    }

    std::vector<Vector3d> TrajectoryGenerator::getDiscreteSolution(Derivative der) {
        std::vector<Vector3d> res;
        if(!isDiscretized_)
            return res;

        for(auto segment = solutionSegments_.begin(); segment != solutionSegments_.end();
            ++segment) {
            auto seg_traj = segment->getTraj(der);
            res.insert(res.end(),
                       seg_traj.begin(),
                       seg_traj.end());
        }
        return res;
    }

    double TrajectoryGenerator::getObjectiveFuncVal() const {
        double val = 0.0;
        for(int i = 0; i < STATE_COUNT; ++i) {
            val += solution_[i].transpose() * H_[i] * solution_[i];
        }
        val /= 0.5;
        return val;
    }

    /***************************************************************************
     *
     *  Setters
     *
     **************************************************************************/

    void TrajectoryGenerator::addConstraint(TrajectoryConstraint tc) {
        if(getNumWaypoints() == 0) {
            assert(tc.getTime() == 0 );
            keyframes_.push_back(tc);
        } else {
            assert(tc.getTime() > keyframes_.back().getTime());
            keyframes_.push_back(tc);
        }
    }

    void TrajectoryGenerator::setArrivalTimes(VectorXd times) {
        assert(times.size() == keyframes_.size());
        assert(times(0) == 0);
        for(int i = 0; i < times.size(); ++i) {
            keyframes_[i].setTime(times(i));
        }
        problemBuilt_ = false;
        isDiscretized_ = false;
    }

    /***************************************************************************
     *
     *  Other functions
     *
     **************************************************************************/

    void TrajectoryGenerator::buildProblem() {
        for(int i = 0; i < states_; ++i) {
            buildCostMatrix(i);
            buildConstraintMatrix(i);
        }
        problemBuilt_ = true;
    }

    bool TrajectoryGenerator::solveProblem(Solver solver) {
        bool result = true;
        if(!problemBuilt_)
            buildProblem();

        VectorXd poly[State::STATE_COUNT][getNumWaypoints()-1];
        for(int i = 0; i < State::STATE_COUNT; ++i){
            result &= solveProblem(i, solver);
            if(result) {
                // slice up the solution vector
                for(int wp = 0; wp < getNumWaypoints()-1; ++wp) {
                    poly[i][wp] = solution_[i].segment(wp*n_coeffs_, n_coeffs_);
                }
            }
        }

        // Got all the solutions, now create trajectory segments
        for(int wp = 0; wp < getNumWaypoints()-1; ++wp) {
            TrajectorySegment ts(keyframes_[wp].getTime(), keyframes_[wp+1].getTime(), 1,
                                 poly[0][wp], poly[1][wp], poly[2][wp]);
            solutionSegments_.push_back(ts);
        }

        return result;
    }

    bool TrajectoryGenerator::solveProblem(int dim, Solver solver) {
        if (!problemBuilt_)
            return false;

        bool result = false;
        switch (solver) {
            case Solver::OOQP:
                result = solveProblemOoqp(dim);
                break;
            case Solver::LINEAR_SOLVE:
                result = solveProblemCholesky(dim);
                break;
            default:
                result = false;
                break;
        }

        return result;
    }

    bool TrajectoryGenerator::solveProblemOoqp(int dim) {
        /**
         *  Don't use ooqpei's quadratic problem formulation module,
         *  the way we developed the equations can be directly sent to
         *  the OoqpEigenInterface::solve function.
         *  Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u
         */
        SparseMatrix<double, RowMajor> Q = getCostMatrix(dim).sparseView();
        // We have no linear term
        VectorXd c = VectorXd::Zero(getCostMatrix(dim).rows());
        SparseMatrix<double, RowMajor> A = getConstraintMatrix(dim).sparseView();
        auto b = getConstraintVector(dim);
        // Empty vectors and matrices for the rest of the params
        Eigen::SparseMatrix<double, Eigen::RowMajor> C;
        Eigen::VectorXd d, f;
        //ooqpei::OoqpEigenInterface::setIsInDebugMode(true);
        //auto started = std::chrono::high_resolution_clock::now();
        bool result = ooqpei::OoqpEigenInterface::solve(Q, c, A, b, C, d, f, solution_[dim]);
        //auto done = std::chrono::high_resolution_clock::now();
        //long time = std::chrono::duration_cast<std::chrono::nanoseconds>(done-started).count();
        //exec_times_.push_back(ooqpei::OoqpEigenInterface::getExecTime());
        //std::cout << "iterations: " << ooqpei::OoqpEigenInterface::getNumIters() << std::endl;
        //std::cout << "exec time: " << time << std::endl;
        return result;
    }

    bool TrajectoryGenerator::solveProblemCholesky(int dim) {
        /**
         * build the KKT conditions
         * We want to solve
         *
         *  [ H A'  [x*       = [b
         *    A 0 ]  lambda*]    c]
         */
        long size = H_[dim].rows() + Aeq_[dim].rows();
        long zero_size = Aeq_[dim].rows();
        MatrixXd K(size, size);
        K << H_[dim], Aeq_[dim].transpose(), Aeq_[dim],
                MatrixXd::Zero(zero_size, zero_size);

        VectorXd rhs(size);
        rhs << VectorXd::Zero(H_[dim].rows()), beq_[dim];
        VectorXd solution = K.partialPivLu().solve(rhs);
        solution_[dim] = solution.head(getNumVars());
        return true;
    }

    void TrajectoryGenerator::buildCostMatrix(int dim) {
        unsigned long h_size = n_coeffs_ * (getNumWaypoints()-1);
        H_[dim] = MatrixXd::Zero(h_size, h_size);
        unsigned long wps = getNumWaypoints()-1;
        for (int wp = 0; wp < wps; ++wp) {
            MatrixXd H = MatrixXd::Zero(n_coeffs_, n_coeffs_);
            for (int i = 0; i <= n_; ++i) {
                for (int j = 0; j <= n_; ++j) {
                    if (i >= k_r_ && j >= k_r_) {
                        double cum_mul = 1;
                        for (int m = 0; m < k_r_; ++m) {
                            cum_mul = cum_mul * (i - m) * (j - m);
                        }
                        H(i, j) = cum_mul / (i+j-(2*k_r_)+1);
                    } else {
                        H(i, j) = 0;
                    }
                }
            }
            double t0 = keyframes_[wp].getTime();
            double tend = keyframes_[wp + 1].getTime();
            H *= 1 / pow((tend - t0), 2*k_r_ - 1);
            H = rot90(H, 2); // Equivalent to rot90(rot90(H)) in matlab
            // Block diagonal insertion
            /**
             * TODO check if I should be using a sparse matrix and if there is a difference
             * and if there is a difference in usage
             */
            unsigned long index = wp * n_coeffs_;
            H_[dim].block(index, index, n_coeffs_, n_coeffs_) = H;
#ifdef DEBUG
            std::cout << "intermediate cost matrix wp " << wp << " \n" << H << std::endl;
#endif
        }
    }

    void TrajectoryGenerator::buildConstraintMatrix(int dim) {
        MatrixXd A_fixed[states_], A_continuity[states_];
        VectorXd b_fixed[states_], b_continuity[states_];
        std::vector<VectorXd> A_eq;
        std::vector<double> b_eq;
        unsigned long wps = getNumWaypoints();
        int constraint_size = (wps-1) * n_coeffs_;
        MatrixXd I = rot90(MatrixXd::Identity(n_coeffs_, n_coeffs_));
        MatrixXd coeffs = genCoefficientMatrix(n_, k_r_);
        for(int wp = 0; wp < wps; ++wp) {
            // We don't go to the snap because that's what we want to minimize I think...
            for(int der = Derivative::DER_POSITION; der < Derivative::DER_SNAP; ++der){
                // Don't add a constraint if it's unconstrained, duh!
                if(!keyframes_[wp].isConstrained(der, dim))
                    continue;

                if(wp == 0){
                    // Initial conditions, only add departure constraints
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double int_t = 1 / std::pow(t_next - t_now, der);

                    VectorXd polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * int_t;
                    unsigned int idx = wp * n_coeffs_;
                    VectorXd a = VectorXd::Zero(constraint_size);
                    a.segment(idx, n_coeffs_) << polynomial;
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    A_eq.push_back(a);
                    b_eq.push_back(b);

                } else if(wp == wps-1) {
                    // Final conditions, only add arrival constraints
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();
                    double int_t_prev = 1 / std::pow(t_now - t_prev, der);

                    VectorXd polynomial = coeffs.row(der) * int_t_prev;
                    unsigned int idx = (wp-1) * n_coeffs_;
                    VectorXd a = VectorXd::Zero(constraint_size);
                    a.segment(idx, n_coeffs_) << polynomial;
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    A_eq.push_back(a);
                    b_eq.push_back(b);
                } else {
                    // Intermediate waypoint, add both departure and arrival constraints
                    VectorXd a1 = VectorXd::Zero(constraint_size);
                    VectorXd a2 = a1;
                    double t_next = keyframes_[wp+1].getTime();
                    double t_now = keyframes_[wp].getTime();
                    double t_prev = keyframes_[wp-1].getTime();

                    // Arrival constraint
                    double int_t_prev = 1 / std::pow(t_now - t_prev, der);
                    VectorXd polynomial = coeffs.row(der) * int_t_prev;
                    unsigned int idx = (wp-1) * n_coeffs_;
                    a1.segment(idx, n_coeffs_) << polynomial;

                    // Departure constraint
                    double int_t = 1 / std::pow(t_next - t_now, der);
                    polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * int_t_prev;
                    idx = wp * n_coeffs_;
                    a2.segment(idx, n_coeffs_) << polynomial;

                    A_eq.push_back(a1);
                    A_eq.push_back(a2);
                    double b = keyframes_[wp].getConstraint(der)(dim);
                    b_eq.push_back(b);  // !!Both a1 and a2 are equal to the same thing
                    b_eq.push_back(b);
                }
            }
        }
        // Dump the vectors into the Eigen matrices
        assert(A_eq.size() == b_eq.size()); //Would be pretty awks if this wasn't true
        A_fixed[dim] = MatrixXd(A_eq.size(), constraint_size);
        b_fixed[dim] = VectorXd(b_eq.size());

        for(int i = 0; i < A_eq.size(); ++i){
            A_fixed[dim].row(i) = A_eq[i];
            b_fixed[dim](i) = b_eq[i];
        }

        A_eq.clear();
        b_eq.clear();

        /**
         * Do the continuity constraints
         * Basically for every unconstrained derivative, make the first
         * polynomial segment equal to the second polynomial segment
         */
        // Purposely skip first and last waypoint, only want intermediate waypoints
        for(int wp = 1; wp < wps-1; ++wp){
            for(int der = Derivative::DER_POSITION; der < Derivative::DER_COUNT; ++der){
                // If the derivative was constrained, we already added it previously
                if(keyframes_[wp].isConstrained(der, dim))
                    continue;
                double t_next = keyframes_[wp+1].getTime();
                double t_now = keyframes_[wp].getTime();
                double t_prev = keyframes_[wp-1].getTime();
                double int_t_prev = 1 / std::pow(t_now - t_prev, der);
                double int_t = 1 / std::pow(t_next - t_now, der);
                VectorXd a = VectorXd::Zero(constraint_size);

                // From prev wp to now
                VectorXd polynomial = coeffs.row(der) * int_t_prev;
                unsigned int idx = (wp-1) * n_coeffs_;
                a.segment(idx, n_coeffs_) << polynomial;

                // from now to next wp
                polynomial = coeffs.row(der).cwiseProduct(I.row(der)) * (-int_t);
                idx = wp * n_coeffs_;
                a.segment(idx, n_coeffs_) << polynomial;
                A_eq.push_back(a);
                b_eq.push_back(0);
            }
        }

        // Dump the vectors into the eigen matrices
        assert(A_eq.size() == b_eq.size()); //Would be pretty awks if this wasn't true
        A_continuity[dim] = MatrixXd(A_eq.size(), constraint_size);
        b_continuity[dim] = VectorXd(b_eq.size());

        for(int i = 0; i < A_eq.size(); ++i) {
            A_continuity[dim].row(i) = A_eq[i];
            b_continuity[dim](i) = b_eq[i];
        }

        // Save matrices and vectors
        Aeq_[dim] = MatrixXd(A_fixed[dim].rows() + A_continuity[dim].rows(),
                             A_fixed[dim].cols());
        Aeq_[dim] << A_fixed[dim], A_continuity[dim];
        beq_[dim] = VectorXd(b_fixed[dim].size() + b_continuity[dim].size());
        beq_[dim] << b_fixed[dim], b_continuity[dim];
    }

    std::vector<Vector3d> TrajectoryGenerator::discretizeSolution() {
        std::vector<Vector3d> fulltraj;
        for(auto segment = solutionSegments_.begin(); segment != solutionSegments_.end();
            ++segment) {
            auto seg_traj = segment->discretize(dt_);
            fulltraj.insert(fulltraj.end(),
                            seg_traj.begin(),
                            seg_traj.end());
        }
        isDiscretized_ = true;
        return fulltraj;
    }
}

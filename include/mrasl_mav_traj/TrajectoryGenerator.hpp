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
 * @file TrajectoryGenerator.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#ifndef MRASL_MAV_TRAJ_TRAJECTORYGENERATOR_HPP
#define MRASL_MAV_TRAJ_TRAJECTORYGENERATOR_HPP

#include <Eigen/Core>
#include "mrasl_mav_traj/TrajectoryConstraint.hpp"
#include "mrasl_mav_traj/TrajectorySegment.hpp"

using namespace Eigen;
namespace mrasl {
    class TrajectoryGenerator {
    public:
        enum Solver {OOQP, BLEIC, LINEAR_SOLVE};
        TrajectoryGenerator();

        /****************************
         * Getters
         ****************************/

        /**
         * Get a copy of the vector of constraints.
         * @return
         */
        std::vector<TrajectoryConstraint> getConstraints();

        /**
         * Get the number of waypoints (constraints) in the trajectory
         * includes the initial conditions.
         * @return unsigned long waypoints
         */
        unsigned long getNumWaypoints();

        /**
         * Get the total number of variables in the optimization problem
         * Should be equal to (k_r + 1) * m.
         * @return
         */
        unsigned long getNumVars();

        /**
         * Get the arrival times of every waypoint, initial conditions  count as
         * a wp at t = 0.
         * @return
         */
        VectorXd getArrivalTimes() const;

        /**
         * Get the cost matrix for a certain dimension of the QP problem.
         * Basically get H in
         * min x'Hx + c'x
         *  x
         * @param dim
         * @return
         */
        MatrixXd getCostMatrix(int dim) const;

        /**
         * Gets the full constraints matrix Aeq including fixed constraints
         * and continuity constraints
         * @param dim
         * @return
         */
        MatrixXd getConstraintMatrix(int dim) const;

        /**
         * Gets the beq vector in Aeq * x = beq. Including fixed
         * constraints and continuity constraints.
         * @param dim
         * @return
         */
        VectorXd getConstraintVector(int dim) const;

        /**
         * Get the raw solution straight from the optimizer
         * Will be a vector containing all the coefficients of the
         * polynomials
         * @param dim
         * @return
         */
        VectorXd getSolution(int dim) const;

        /**
         * Get the discrete solution for a certain derivative of the trajectory
         * @param der
         * @return
         */
        std::vector<Vector3d> getDiscreteSolution(Derivative der);

        /**
         * Once a solution is found, compute the value of the objective
         * function we minimized.
         * @return
         */
        double getObjectiveFuncVal() const;

        /****************************
         * Setters
         ****************************/

        /**
        * Add a constraint to the trajectory.
        * @param tc The constraint
        */
        void addConstraint(TrajectoryConstraint tc);

        /**
         * Changes all the waypoint arrival times (expect wp_0 that's always 0).
         * @param times
         */
        void setArrivalTimes(VectorXd times);

        /****************************
         * Other functions
         ****************************/

        /**
         * Build the required matrices for the QP problem using the
         * constraints currently added to the generator.
         */
        void buildProblem();

        /**
         * Send off the problem in all states to a solver and run optimization
         * Currently the working solvers are OOQP, Gurobi and qpOASES
         * @param solver
         * @return
         */
        bool solveProblem(Solver solver);

        /**
         * Send off the problem to a solver and run the optimization
         * algorithm
         * @param dim
         * @return If the optimization worked
         */
        bool solveProblem(int dim, Solver solver);

    private:
        // TODO make k_r and n_ parameters?
        static constexpr int k_r_ = Derivative::DER_SNAP;   // Order of the
        // derivative of the
        // position
        static constexpr int n_ = 6;     // Order of the polynomials describing the trajectory
        static constexpr int n_coeffs_ = n_ + 1;   // Number of coefficients in a
                                            // polynomial
        static constexpr int states_ = State::STATE_COUNT;// Number of states
        // in the problem
        static constexpr double dt_ = 0.1;
        std::vector<TrajectoryConstraint> keyframes_;       // Constraints on the trajectory
        // including initial conditions
        MatrixXd H_[states_];           // Cost matrices
        MatrixXd Aeq_[states_];         // Equality constraint matrix
        VectorXd beq_[states_];         // Equality constraint vector
        VectorXd solution_[states_];    // Solution vector
        std::vector<TrajectorySegment> solutionSegments_;   // Polynomial segments

        bool problemBuilt_;
        bool isDiscretized_;

        /****************************
         * Solver calls
         ****************************/
        bool solveProblemOoqp(int dim);
        bool solveProblemBLEIC(int dim);
        bool solveProblemLU(int dim);

        /**
         * Build the cost matrix for a certain dimension dim
         * and store it in H_. Follows the equations outlined in
         * Adam Bry's thesis and richter_rss13_workshop
         * @param dim
         */
        void buildCostMatrix(int dim);

        /**
         * Build the constraint matrix for the QP problem
         * any constraints marked with std::numeric_limits<double>::max()
         * will be treated as unconstrained
         * @param dim
         */
        void buildConstraintMatrix(int dim);

        /**
         * Discretize the solution into x, y and z vectors with dt.
         * @return
         */
        std::vector<Vector3d> discretizeSolution();
    };
}

#endif //MRASL_MAV_TRAJ_TRAJECTORYGENERATOR_HPP

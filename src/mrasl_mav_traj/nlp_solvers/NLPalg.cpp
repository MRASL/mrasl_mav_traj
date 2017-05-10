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
 * @file NLPalg.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <mrasl_mav_traj/TimeAllocator.hpp>
#include <iostream>
#include "mrasl_mav_traj/TrajectoryMath.hpp"
#include "mrasl_mav_traj/nlp_solvers/NLPalg.hpp"

namespace mrasl {
    int NLPalg::func_evals_ = 0;

    NLPalg::NLPalg(generator_solver_pair_t pair) : TimeAllocator(pair){}

    bool NLPalg::optimize() {
        /**
         * Where f(x) is the value of the objective function used
         * for trajectory generation, we want solve
         *      min f(x)
         *      s.t. sum(Ti) = t_final
         *           Ti >= 0
         * Where Ti are the segment times from waypoint t_i-1 to t_i
         *
         * To translate to the alglib formulation we have
         * x -> segment times
         * c -> 1 row all ones = 0
         * li -> lower bound constraint = 0
         */
        // 1. Create problem
        auto times = pair_.tg->getArrivalTimes();
        alglib::real_1d_array x = eigen2alg(time2segment(times));
        alglib::minbleicstate state;
        alglib::minbleiccreate(x, state);

        // 2. Set linear constraints, sum of segment times = last arrival time
        double ones[x.length() + 1];
        std::fill_n(ones, x.length(), 1.0);
        ones[x.length()] = times(times.size()-1);   // Last element is the rhs
        alglib::real_2d_array c;
        c.setcontent(1, x.length()+1, ones);

        alglib::integer_1d_array ct = "[0]";  // equality constraint
        alglib::minbleicsetlc(state, c, ct);

        // 3. Set bounds constraints lc = 0 uc = inf
        //double zeros[x.length()]

        // 4. Set optimization termination conditions
        double epsg = 1e-6;
        double epsf = 0;
        double epsx = 0;
        alglib::ae_int_t maxits = 0;
        alglib::minbleicsetcond(state, epsg, epsf, epsx, maxits);

        // 5. Optimize!
        generator_solver_pair_t* gsp = &pair_;
        alglib::minbleicoptimize(state, NLPalg::timeAllocGrad, NULL,
                                 (void*)gsp);

        // 6. Get results
        alglib::minbleicreport rep;
        alglib::minbleicresults(state, x, rep);
#ifdef DEBUG
        std::cout << "ALG func evals " << func_evals_ << std::endl;
        std::cout << "term type " << (int)rep.terminationtype << std::endl;
        std::cout << "iter cnt  " << (int)rep.iterationscount << std::endl;
        std::cout << "solution\n" << alg2eigen(x) << std::endl;
        std::cout << "solution times\n" << segment2time(alg2eigen(x)) << std::endl;
#endif

        return (rep.terminationtype > 0);
    }

    void NLPalg::timeAllocGrad(const alglib::real_1d_array &x, double &func,
                                          alglib::real_1d_array &grad, void *ptr) {
        const double h_ = 0.001;
        generator_solver_pair_t* gsp = (generator_solver_pair_t*)ptr;
        auto tg = gsp->tg;
        auto solver = gsp->solver;

        // Evaluate trajectory with current time distribution
        //std::cout << "alg2eig \n" << alg2eigen(x) << std::endl;
        //std::cout << "seg2time \n" << segment2time(alg2eigen(x)) << std::endl;
        VectorXd arrival_times = segment2time(alg2eigen(x));
        tg->setArrivalTimes(arrival_times);
        tg->solveProblem(solver);
        func = tg->getObjectiveFuncVal();

        // Evaluate gradient
        long m = tg->getNumWaypoints() - 1; // Match matlab code :-)
        MatrixXd gi = generateGi(m);
        alglib::real_1d_array nabla;
        nabla.setlength(m);
        for(int i = 0; i < m; ++i) {
            VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
            tg->setArrivalTimes(t);
            tg->solveProblem(solver);
            nabla(i) = (tg->getObjectiveFuncVal() - func) / h_;
        }
        grad = nabla;
        func_evals_++;
    }

    alglib::real_1d_array NLPalg::eigen2alg(const Eigen::VectorXd vec) {
        alglib::real_1d_array r;
        r.setcontent(vec.size(), vec.data());
        return r;
    }

    alglib::real_2d_array NLPalg::eigen2alg(const Eigen::MatrixXd mat) {
        alglib::real_2d_array r;
        r.setlength(mat.rows(), mat.cols());
        for(int j = 0; j < mat.cols(); ++j) {
            for(int i = 0; i < mat.rows(); ++i) {
                r(i,j) = mat(i,j);
            }
        }
        return r;
    }

    Eigen::VectorXd NLPalg::alg2eigen(const alglib::real_1d_array r) {
        Eigen::VectorXd v(r.length());
        for(int i = 0; i < r.length(); ++i)
            v(i) = r(i);
        return v;
    }
}
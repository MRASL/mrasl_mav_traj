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
 * @file NLPnlopt.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <iostream>
#include <nlopt.hpp>
#include <mrasl_mav_traj/TimeAllocator.hpp>
#include "mrasl_mav_traj/TimeAllocator.hpp"
#include "mrasl_mav_traj/TrajectoryMath.hpp"
#include "mrasl_mav_traj/nlp_solvers/NLPnlopt.hpp"

namespace mrasl {
  NLPnlopt::NLPnlopt(generator_solver_pair_t pair) : TimeAllocator(pair) {}

  bool NLPnlopt::optimize() {
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
    auto segment_times = time2segment(times);
    nlopt::opt opt(nlopt::algorithm::LD_SLSQP, segment_times.rows());
    generator_solver_pair_t* gsp = &pair_;
    opt.set_min_objective(nlopt_obj_func, (void*)gsp);

    // 2. Set linear constraints, sum of segment times = last arrival time
    opt.set_lower_bounds(0);
    double last = times(times.size()-1);
    opt.add_equality_constraint(nlopt_constr, (void*)&last, 1e-8);
    //double lb[segment_times.rows()];
    //memset(lb, 0, segment_times.rows());


    // 3. Set bounds constraints lc = 0 uc = inf

    // 4. Set optimization termination conditions
    opt.set_xtol_rel(1e-8);
    opt.set_maxeval(40);

    // 5. Optimize!
    std::vector<double> t(segment_times.rows());
    for(auto it = t.begin(); it != t.end(); ++it)
      *it = 1;
    double minf;
    nlopt::result res = opt.optimize(t, minf);
#ifdef DEBUG
    switch(res) {
      case nlopt::SUCCESS:
        std::cout << "success" << std::endl;
        break;
      case nlopt::STOPVAL_REACHED:
        std::cout << "STOPVAL_REACHED" << std::endl;
        break;
      case nlopt::FTOL_REACHED:
        std::cout << "FTOL_REACHED" << std::endl;
        break;
      case nlopt::XTOL_REACHED:
        std::cout << "XTOL_REACHED" << std::endl;
        break;
      case nlopt::MAXEVAL_REACHED:
        std::cout << "MAXEVAL_REACHED" << std::endl;
        break;
      case nlopt::MAXTIME_REACHED:
        std::cout << "MAXTIME_REACHED" << std::endl;
        break;
      default:
        std::cout << "failed" << std::endl;
        break;
    }

    std::cout << "result " << (res > 0 ? "worked!\n" : "failed\n");
    for(auto it = t.begin(); it != t.end(); ++it)
      std::cout << *it << ", ";
    std::cout << "\nminf " << minf << std::endl;
    std::cout << "iters " << gsp->iters << std::endl;
#endif
    return (res > 0);
  }

  double NLPnlopt::nlopt_obj_func(const std::vector<double> &x,
                                  std::vector<double> &grad, void *f_data) {
    const double h_ = 0.001;
    generator_solver_pair_t *gsp = (generator_solver_pair_t *) f_data;
    auto tg = gsp->tg;
    auto solver = gsp->solver;
    gsp->iters += 1;

    //Evaluate trajectory with current time distribution
    VectorXd times(x.size());
    for (int i = 0; i < x.size(); ++i) {
      times(i) = x[i];
    }
    VectorXd arrival_times = segment2time(times);
    tg->setArrivalTimes(arrival_times);
    tg->solveProblem(solver);
    auto value = tg->getObjectiveFuncVal();

    // Evaluate gradient
    if (!grad.empty()) {
      long m = tg->getNumWaypoints() - 1; // Match matlab code :-)
      MatrixXd gi = generateGi(m);
      for (int i = 0; i < m; ++i) {
        VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
        tg->setArrivalTimes(t);
        tg->solveProblem(solver);
        grad[i] = (tg->getObjectiveFuncVal() - value) / h_;
      }
    }

    return value;
  }

  double NLPnlopt::nlopt_constr(const std::vector<double> &x,
                                std::vector<double> &grad, void *data) {
    double last = *((double*)data);
    if(!grad.empty()){
      for(auto it = grad.begin(); it != grad.end(); ++it)
        *it = 1;
    }

    double sum = 0.0;
    for(auto it = x.begin(); it != x.end(); ++it) {
      sum += *it;
    }
    return sum - last;
  }
}

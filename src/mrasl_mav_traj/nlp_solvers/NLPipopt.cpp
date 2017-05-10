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
 * @file NLPipopt.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <cassert>
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "mrasl_mav_traj/nlp_solvers/NLPipopt.hpp"


 using namespace Ipopt;

namespace mrasl {
  NLPipopt::NLPipopt(generator_solver_pair_t pair) : TimeAllocator(pair) {}

  bool NLPipopt::optimize() {
    SmartPtr<TNLP> nlp = this;
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
    }
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("limited_memory_update_type", "bfgs");
    status = app->OptimizeTNLP(nlp);
  }

  bool NLPipopt::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = pair_.tg->getNumWaypoints() - 1;
    m = 1;
    nnz_jac_g = n;
    nnz_h_lag = n;  // Don't know
    index_style = C_STYLE;
    return true;
  }

  bool NLPipopt::get_bounds_info( Ipopt::Index n, Number *x_l, Number *x_u, Ipopt::Index m,
                                  Number *g_l, Number *g_u) {
    assert(n == pair_.tg->getNumWaypoints() - 1);
    assert(m == 1);

    // All segment times have to be positive
    for(int i = 0; i < n; ++i) {
      x_l[i] = 0.0;
      x_u[i] = IPOPT_INFINITY;
    }

    // Sum of segment times has to equal last arrival time
    g_l[0] = g_u[0] = pair_.tg->getArrivalTimes()(n);

    return true;
  }

  bool NLPipopt::get_starting_point(Ipopt::Index n, bool init_x, Number *x,
                          bool init_z, Number *z_L, Number *z_U,
                          Ipopt::Index m, bool init_lambda, Number *lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    // start from the current time distribution
    VectorXd arrival_times = pair_.tg->getArrivalTimes();
    VectorXd segment_times = time2segment(arrival_times);
    for(int i = 0; i < n; ++i) {
      x[i] = segment_times(i);
    }

    return true;
  }

  bool NLPipopt::eval_f(Ipopt::Index n, const Number *x, bool new_x,
                        Number& obj_value) {
    VectorXd segment_times(n);
    for(int i = 0; i < n; ++i) {
      segment_times(i) = x[i];
    }
    VectorXd arrival_times = segment2time(segment_times);
    pair_.tg->setArrivalTimes(arrival_times);
    pair_.tg->solveProblem(pair_.solver);
    obj_value = pair_.tg->getObjectiveFuncVal();
  }

  bool NLPipopt::eval_grad_f( Ipopt::Index n, const Number *x, bool new_x,
                              Number *grad_f) {
    // Start by evaluating where we are and saving data
    VectorXd T(n);
    for(int i = 0; i < n; ++i) {
      T(i) = x[i];
    }
    VectorXd arrival_times = segment2time(T);
    pair_.tg->setArrivalTimes(arrival_times);
    pair_.tg->solveProblem(pair_.solver);
    double orig_fun_val = pair_.tg->getObjectiveFuncVal();

    // Evaluate gradient
    MatrixXd gi = generateGi(n);
    for(int i = 0; i < n; ++i) {
      VectorXd t = segtimeRealloc(arrival_times, h_ * gi.col(i));
      pair_.tg->setArrivalTimes(t);
      pair_.tg->solveProblem(pair_.solver);
      grad_f[i] = (pair_.tg->getObjectiveFuncVal() - orig_fun_val) / h_;
    }

    return true;
  }

  bool NLPipopt::eval_g(Ipopt::Index n, const Number *x, bool new_x, Ipopt::Index m,
                        Number *g) {
    double sum = 0.0;
    for(int i = 0; i < n; ++i) {
      sum += x[i];
    }
    g[0] = sum;

    return true;
  }

  bool NLPipopt::eval_jac_g(Ipopt::Index n, const Number *x, bool new_x, Ipopt::Index m,
                            Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol,
                            Number *values) {
    // [ 1 1 1 ...]
    if(values == NULL) {
      for(int i = 0; i < n; ++i) {
        iRow[i] = 0;
        jCol[i] = i;
      }
    } else {
      for(int i = 0; i < n; ++i) {
        values[i] = 1;
      }
    }
  }

  bool NLPipopt::eval_h(Ipopt::Index n, const Number *x, bool new_x,
              Number obj_factor, Ipopt::Index m, const Number *lambda,
              bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
              Ipopt::Index *jCol, Number *values) {
    // https://www.coin-or.org/Ipopt/documentation/node31.html
    return false;
  }

  void NLPipopt::finalize_solution(SolverReturn status, Ipopt::Index n, const Number *x,
    const Number *z_L, const Number *z_U, Ipopt::Index m, const Number *g,
    const Number *lambda, Number obj_value, const IpoptData *ip_data,
    IpoptCalculatedQuantities *ip_cq) {

  }
}

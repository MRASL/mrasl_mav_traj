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
 * @file NLPipopt.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 * @description Implements non-linear solving of the time allocation problem
 * using the ipopt interior point solver.
 **/

 #ifndef MRASL_MAV_TRAJ_NLPIPOPT_HPP
 #define MRASL_MAV_TRAJ_NLPIPOPT_HPP


#include "IpTNLP.hpp"
#include "mrasl_mav_traj/TimeAllocator.hpp"

using namespace Ipopt;

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
 *
 * IPOPT worls the following way:
 *    min     f(x)
 *  x in R^n
 *  s.t.        g_L <= g(x) <= g_U
 *              x_L <=  x   <= x_U
 */
class NLPipopt : public TimeAllocator, public TNLP {
public:
  NLPipopt(generator_solver_pair_t pair);
  bool optimize();

  /**@name Overloaded from TNLP */
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Ipopt::Index &n,
                            Ipopt::Index &m,
                            Ipopt::Index &nnz_jac_g,
                            Ipopt::Index &nnz_h_lag,
                            IndexStyleEnum &index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Ipopt::Index n,
                               Number *x_l,
                               Number *x_u,
                               Ipopt::Index m,
                               Number *g_l,
                               Number *g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Ipopt::Index n,
                                  bool init_x,
                                  Number *x,
                                  bool init_z,
                                  Number *z_L,
                                  Number *z_U,
                                  Ipopt::Index m,
                                  bool init_lambda,
                                  Number *lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Ipopt::Index n,
                      const Number *x,
                      bool new_x,
                      Number &obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Ipopt::Index n,
                           const Number *x,
                           bool new_x,
                           Number *grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Ipopt::Index n,
                      const Number *x,
                      bool new_x,
                      Ipopt::Index m,
                      Number *g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Ipopt::Index n,
                          const Number *x,
                          bool new_x,
                          Ipopt::Index m,
                          Ipopt::Index nele_jac,
                          Ipopt::Index *iRow,
                          Ipopt::Index *jCol,
                          Number *values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Ipopt::Index n,
                      const Number *x,
                      bool new_x,
                      Number obj_factor,
                      Ipopt::Index m,
                      const Number *lambda,
                      bool new_lambda,
                      Ipopt::Index nele_hess,
                      Ipopt::Index *iRow,
                      Ipopt::Index *jCol,
                      Number *values);

  // @}

  /** @name Solution Methods */

  // @{

  /** This method is called when the algorithm is complete so the TNLP can
     store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Ipopt::Index n,
                                 const Number *x,
                                 const Number *z_L,
                                 const Number *z_U,
                                 Ipopt::Index m,
                                 const Number *g,
                                 const Number *lambda,
                                 Number obj_value,
                                 const IpoptData *ip_data,
                                 IpoptCalculatedQuantities *ip_cq);

  // @}

private:
  const double IPOPT_INFINITY = 1.0e19;
};
}

 #endif // MRASL_MAV_TRAJ_NLPIPOPT_HPP

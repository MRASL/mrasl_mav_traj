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
 * @file Solvers.hpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

 #ifndef SOLVERS_HPP
 #define SOLVERS_HPP

// ALG is always supported
 #include "mrasl_mav_traj/nlp_solvers/NLPalg.hpp"

 #ifdef USE_OOQP

 #endif // USE_OOQP
 #ifdef USE_GUROBI

 #endif // USE_GUROBI
 #ifdef USE_GUROBI

 #endif // USE_GUROBI
 #ifdef USE_QPOASES

 #endif // USE_QPOASES
 #ifdef USE_NLOPT
# include "mrasl_mav_traj/nlp_solvers/NLPnlopt.hpp"
 #endif // USE_NLOPT

 #endif // SOLVERS_HPP

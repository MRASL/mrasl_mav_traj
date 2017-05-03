# mrasl_mav_traj

Trajectory utilities for MAVs

The trunk of this work was done by Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca> 
as part of the class MTH8404: Méthodes d'optimisation et contrôle optimal.

Currently it is a reimplementation of the trajectory generation part of the paper 
*Minimum Snap Trajectory Generation and Control for Quadrotors* by Daniel 
Mellinger and Vijay Kumar [3]. With inspiration from Adam P. Bry's PhD thesis [1] and a paper
by Richter and al. on Polynomial Trajectory Planning [2].

This package supports multiple Quadratic Programming and Non-Linear Programming solvers for comparison
purposes including OOQP, qpOASES, Ipopt, ALGLIB's BLEIC and Gurobi. For development purposes, a first implementation
war written in Matlab using the quadprog and fmincon solvers.

# Building
TODO

# Usage
TODO

# References
1. A. P. Bry. *Control, estimation, and planning algorithms for aggressive flight using onboard sensing*. PhD thesis, Citeseer, 2012.
1. C. Richter, A. Bry, and N. Roy. *Polynomial Trajectory Planning for Aggressive Quadrotor Flight in
Dense Indoor Environments*, pages 649–666. Springer International Publishing, Cham, 2016.
1. D. Mellinger and V. Kumar. *Minimum snap trajectory generation and control for quadrotors*. In *2011
IEEE International Conference on Robotics and Automation*, pages 2520–2525, May 2011.

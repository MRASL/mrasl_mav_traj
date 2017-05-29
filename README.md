# mrasl_mav_traj

Trajectory utilities for MAVs

The trunk of this work was done by Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
as part of the class MTH8404: Méthodes d'optimisation et contrôle optimal.

Currently it is a reimplementation of the trajectory generation part of the
paper *Minimum Snap Trajectory Generation and Control for Quadrotors* by Daniel
Mellinger and Vijay Kumar [3]. With inspiration from Adam P. Bry's PhD
thesis [1] and a paper by Richter and al. on Polynomial Trajectory Planning [2].

This package supports multiple Quadratic Programming and Non-Linear Programming
solvers for comparison purposes including OOQP, qpOASES, Ipopt, ALGLIB's BLEIC
and Gurobi. For development purposes, a first implementation was written in
Matlab using the quadprog and fmincon solvers.

# Building
Before building, you have to pick which solvers you are going to use. The `CMakeLists.txt` file contains options at the top for supported solvers that you might need to install if you want to use them. The best conbination of solvers is probably the Eigen's LU for the simple solve and IPOPT for the time allocation.

# Usage
There are example usages in the tests folder. 

## Building the problem
For example, if you want to build a slalom path
```c++
Vector3d wp1, wp2, wp3, wp4, wp5, wp6, wp7;
wp1 << 0,   0,  1.5;
wp2 << 1,   1,  1;
wp3 << 0,   2,  2;
wp4 << -1,  3,  1;
wp5 << 0,   4,  2;
wp6 << 1,   5,  1;
wp7 << 0,   6,  1.5;

TrajectoryConstraint tc1(0, wp1, Vector3d::Zero(), Vector3d::Zero(),
                         Vector3d::Zero(), Vector3d::Zero());
TrajectoryConstraint tc2(1, wp2);
TrajectoryConstraint tc3(2, wp3);
TrajectoryConstraint tc4(3, wp4);
TrajectoryConstraint tc5(4, wp5);
TrajectoryConstraint tc6(5, wp6);
TrajectoryConstraint tc7(6, wp7, Vector3d::Zero(), Vector3d::Zero(),
                         Vector3d::Zero(), Vector3d::Zero());

TrajectoryGenerator *tg = new TrajectoryGenerator();
tg->addConstraint(tc1);
tg->addConstraint(tc2);
tg->addConstraint(tc3);
tg->addConstraint(tc4);
tg->addConstraint(tc5);
tg->addConstraint(tc6);
tg->addConstraint(tc7);
```
In this example we constrain the start and end positions to have zero velocity, acceleration, jerk and snap, all other waypoints are only constrained in position. The first argument of the `TrajectoryConstraint` constructor is the arrival time of the waypoint.

## Simple solve
One you have built your problem you can solve it by calling:
```c++
tg->solveProblem(TrajectoryGenerator::Solver::LINEAR_SOLVE);
```
The solveProblem function takes as input a solving method, since our quadratic problems are only equality constrained, it is possible to solve them using LU decomposition instead of optimization. Nonetheless, OOQP and ALGLIB's BLEIC are also supported.

## Solve with time optimal segments
Mellinger shows that you can keep the total flight time and optimize the time allocation for each segment between waypoints. We support 3 solvers for this problem: IPOPT, NLopt and ALGLIB's BLEIC. For example:
```c++
Vector3d wp0, wp1, wp2, wp3;
wp0 << 0, 0, 0;
wp1 << 1, 0, 0;
wp2 << 1, 2, 0;
wp3 << 0, 2, 0;

TrajectoryGenerator *tg = new TrajectoryGenerator();
tg->addConstraint(
    TrajectoryConstraint(0, wp0, Vector3d::Zero(), Vector3d::Zero(),
                         Vector3d::Zero(), Vector3d::Zero()));
tg->addConstraint(TrajectoryConstraint(0.5, wp1));
tg->addConstraint(TrajectoryConstraint(2.5, wp2));
tg->addConstraint(
  TrajectoryConstraint(3, wp3, Vector3d::Zero(), Vector3d::Zero(),
                       Vector3d::Zero(), Vector3d::Zero()));

TimeAllocator::generator_solver_pair_t gsp;
gsp.tg = tg;
gsp.solver = TrajectoryGenerator::Solver::LINEAR_SOLVE;
TimeAllocator *ta = new NLPnlopt(gsp);
ta->optimize();
VectorXd solution = tg->getArrivalTimes();
```

# Todo
* Support for scale factor
  * Mellinger shows that the solution we get after optimization is for nondimentionalized time, so you can take the end solution and scale it in time if you want to reduce velocity/acceleration/jerk/snap demanded on the vehicle. The Matlab implementation supports this but the C++ doesn't. To support this we need to allow the `TimeSegment` class to change the value of `alpha_` after construction and add the time scale coefficient as a parameter to all discretization functions in `TimeSegment` and `TrajectoryGenerator`.

# References
1. A. P. Bry. *Control, estimation, and planning algorithms for aggressive flight using onboard sensing*. PhD thesis, Citeseer, 2012.
1. C. Richter, A. Bry, and N. Roy. *Polynomial Trajectory Planning for Aggressive Quadrotor Flight in
Dense Indoor Environments*, pages 649–666. Springer International Publishing, Cham, 2016.
1. D. Mellinger and V. Kumar. *Minimum snap trajectory generation and control for quadrotors*. In *2011
IEEE International Conference on Robotics and Automation*, pages 2520–2525, May 2011.

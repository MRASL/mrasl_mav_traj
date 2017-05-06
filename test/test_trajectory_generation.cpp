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
 * @file test_trajectory_generation.cpp
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 **/

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <mrasl_mav_traj/TrajectoryMath.hpp>
#include <mrasl_mav_traj/TrajectoryGenerator.hpp>
#include <mrasl_mav_traj/TrajectoryConstraint.hpp>

using namespace mrasl;
using namespace Eigen;

TEST(traj_generation, figure8_linear) {
    Vector3d wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8, wp9;
    wp1 << 0, 0, 1;
    wp2 << 2, -2, 1.5;
    wp3 << 4, 0, 2;
    wp4 << 2, 2, 1.5;
    wp5 << 0, 0, 1;
    wp6 << -2, -2, 1.5;
    wp7 << -4, 0, 2;
    wp8 << -2, 2, 1.5;
    wp9 << 0, 0, 1;

    TrajectoryConstraint tc1(0, wp1, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());
    TrajectoryConstraint tc2(1, wp2);
    TrajectoryConstraint tc3(2, wp3);
    TrajectoryConstraint tc4(3, wp4);
    TrajectoryConstraint tc5(4, wp5);
    TrajectoryConstraint tc6(5, wp6);
    TrajectoryConstraint tc7(6, wp7);
    TrajectoryConstraint tc8(7, wp8);
    TrajectoryConstraint tc9(8, wp9, Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero());

    TrajectoryGenerator *tg = new TrajectoryGenerator();
    tg->addConstraint(tc1);
    tg->addConstraint(tc2);
    tg->addConstraint(tc3);
    tg->addConstraint(tc4);
    tg->addConstraint(tc5);
    tg->addConstraint(tc6);
    tg->addConstraint(tc7);
    tg->addConstraint(tc8);
    tg->addConstraint(tc9);

    tg->solveProblem(TrajectoryGenerator::Solver::LINEAR_SOLVE);
    VectorXd solution = tg->getSolution(0);
    VectorXd expected[3];
    expected[0] = VectorXd(solution.rows());
    expected[0] << 2.33699778351827, -8.26653984308639, 7.92954205956812, 0, 0,
    0, 0, -0.262173774569666, 0.443362296616624, 1.65180959691016,
    -4.20727452222609, -0.0331793206811688, 4.40745572395014, 2,
    0.119961161205798, -0.373238743779628, -0.0639855385517090, 1.59011134018747,
    -2.24312895827723, -1.02971926078470, 4, -0.0206473752856702,
    0.0880944937295838, -0.130761839362884, 0.00100497230030583, 0.210321811265607,
    -2.14801206264694, 2, 0.0206473752856577, -0.0357897579843964,
    -1.97619698383278e-14, -0.0540449535687977, 5.32907051820075e-15,
    -1.93081266373245, 0, -0.119961161205804, 0.346528223455177,
    0.130761839362865, 0.00100497230031404, -0.210321811265606,
    -2.14801206264695, -2, 0.262173774569652, -1.12968035080133,
    0.0639855385516871, 1.59011134018746, 2.24312895827723, -1.02971926078470,
    -4, -2.33699778351827, 5.75544685802323, -1.65180959691019,
    -4.20727452222608, 0.0331793206811771, 4.40745572395014, -2;

    ASSERT_TRUE(solution.isApprox(expected[0]));

    solution = tg->getSolution(1);
    expected[1] = VectorXd(solution.rows());
    expected[1] << -3.71866243771124, 12.2662483671913, -10.5475859294800, 0,
            0, 0, 0, 0.476725006708453, -0.00529056841649545, -4.99628065919233,
            6.09889119976772, 3.59703152936388, -3.17107650823122, -2,
            -0.304467419854674, 0.398537506326729, 2.12814159935199,
            -4.40463698699751, -0.986009410025129, 5.16843471119859, 0,
            0.147628836863758, -0.353650077224378, -0.446182166834471,
            2.00395607658426, -2.01270700945854, -1.33904565993063, 2,
            -0.147628836863753, 0.532122943958157, 3.99680288865056e-15,
            -0.364696625722254, -2.22044604925031e-16, -2.01979748137215, 0,
            0.304467419854677, -1.42826701280133, 0.446182166834488,
            2.00395607658427, 2.01270700945853, -1.33904565993064, -2,
            -0.476725006708464, 2.85505947183425, -2.12814159935200,
            -4.40463698699753, 0.986009410025133, 5.16843471119860, 0,
            3.71866243771123, -10.0457262590762, 4.99628065919230,
            6.09889119976772, -3.59703152936387, -3.17107650823122, 2;

    ASSERT_TRUE(solution.isApprox(expected[1]));

    solution = tg->getSolution(2);
    expected[2] = VectorXd(solution.rows());
    expected[2] << 0.611914141574227, -2.13865809925612, 2.02674395768190, 0,
            0, 0, 1, -0.0616917431126940, 0.0530834274112310, 0.512165585014687,
            -1.04132233034911, -0.0474051228564487, 1.08517018389233,
            1.50000000000000, 0.0220879710808839, 0.00396454279349434,
            -0.147793424619568, 0.304339421568075, -0.492920476393741,
            -0.189678034429143, 2, 0.000101182918262555, -0.100998965325233,
            0.203348855561164, 0.194570572642427, -0.0956977652587230,
            -0.701323880537896, 1.50000000000000, 0.000101182918260223,
            0.100391867815663, -0.300128227291064, -2.22044604925031e-15,
            0.699635176557142, 9.99200722162641e-16, 1, 0.0220879710808816,
            -0.136492369278789, 0.203348855561154, -0.194570572642426,
            -0.0956977652587192, 0.701323880537898, 1.50000000000000,
            -0.0616917431126933, 0.317067031264932, -0.147793424619569,
            -0.304339421568072, -0.492920476393743, 0.189678034429144, 2,
            0.611914141574229, -1.53282675018924, 0.512165585014690,
            1.04132233034911, -0.0474051228564476, -1.08517018389233,
            1.50000000000000;

    ASSERT_TRUE(solution.isApprox(expected[2]));

    tg->solveProblem(TrajectoryGenerator::Solver::OOQP);
    solution = tg->getSolution(0);
    ASSERT_TRUE(solution.isApprox(expected[0]));
    solution = tg->getSolution(1);
    ASSERT_TRUE(solution.isApprox(expected[1]));
    solution = tg->getSolution(2);
    ASSERT_TRUE(solution.isApprox(expected[2]));

    tg->solveProblem(TrajectoryGenerator::Solver::BLEIC);
    solution = tg->getSolution(0);
    // Just accept that BLEIC isn't as good
    ASSERT_TRUE(solution.isApprox(expected[0], 1e-6));
    solution = tg->getSolution(1);
    ASSERT_TRUE(solution.isApprox(expected[1], 1e-6));
    solution = tg->getSolution(2);
    ASSERT_TRUE(solution.isApprox(expected[2], 1e-6));

    delete tg;
}

TEST(traj_generation, slalom) {
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

    VectorXd expected[3];
    expected[0] = VectorXd(tg->getNumVars());
    expected[1] = VectorXd(tg->getNumVars());
    expected[2] = VectorXd(tg->getNumVars());

    expected[0] << 1.98802329207522, -6.46774460530985, 5.47972131323463, 0,
            0, 0, 0, -0.216375593348483, -0.276410302087895, 2.96134766781372,
            -2.99809495865554, -1.97876879256240, 1.50830197884059, 1,
            0.0871706322847135, 0.316625750096697, -1.66633774285300,
            1.75568082475075, 0.785295417247137, -2.27843488152630, 0,
            0.0871706322847123, -0.839649543804976, 1.22435049190119,
            7.77156117237610e-16, 0.528128419619073, -4.44089209850063e-16,
            -1, -0.216375593348488, 1.57466386217881, -1.66633774285300,
            -1.75568082475075, 0.785295417247138, 2.27843488152630, 0,
            1.98802329207522, -5.46039514714148, 2.96134766781371,
            2.99809495865554, -1.97876879256240, -1.50830197884059, 1;
    expected[1] << 1.33247586162517, -4.57197116704750, 4.23949530542233, 0,
            0, 0, 0, -0.162725640895486, 0.110782819976111, 1.36677739456235,
            -2.11221321628232, -0.295601913563494, 2.09298055620283, 1,
            0.118615276452688, -0.251807205560184, -0.520193118989394,
            1.20821174381847, 0.235366391292481, 0.209806912985941, 2,
            -0.118615276452688, 0.459884453155940, 1.55431223447522e-15,
            -1.01832725868719, 0, 1.67705808198394, 3, 0.162725640895481,
            -0.865571025396795, 0.520193118989389, 1.20821174381846,
            -0.235366391292480, 0.209806912985944, 4, -1.33247586162518,
            3.42288400270353, -1.36677739456237, -2.11221321628232,
            0.295601913563500, 2.09298055620283, 5;
    expected[2] << -1.62729283669913, 4.94342083482999, -3.81612799813086, 0,
            0, 0, 1.50000000000000, 0.127085148963678, 0.940608633250720,
            -3.50841637446783, 1.62383962179389, 2.12804780902780,
            -0.311164838568268, 1, -0.0455202905241383, -1.10311737301844,
            3.10090402624095, -0.462036564296648, -2.73856800543510,
            0.248338207033379, 2, -0.0455202905241372, 1.37623911616327,
            -3.09748719671331, -1.33226762955019e-15, 2.76676837107419,
            1.22124532708767e-15, 1, 0.127085148963682, -1.70311952703280,
            3.10090402624096, 0.462036564296653, -2.73856800543511,
            -0.248338207033382, 2, -1.62729283669912, 4.82033618536477,
            -3.50841637446782, -1.62383962179389, 2.12804780902780,
            0.311164838568268, 1;

    tg->solveProblem(TrajectoryGenerator::Solver::LINEAR_SOLVE);
    VectorXd solution = tg->getSolution(0);
    ASSERT_TRUE(solution.isApprox(expected[0]));
    solution = tg->getSolution(1);
    ASSERT_TRUE(solution.isApprox(expected[1]));
    solution = tg->getSolution(2);
    ASSERT_TRUE(solution.isApprox(expected[2]));

    tg->solveProblem(TrajectoryGenerator::Solver::OOQP);
    solution = tg->getSolution(0);
    ASSERT_TRUE(solution.isApprox(expected[0]));
    solution = tg->getSolution(1);
    ASSERT_TRUE(solution.isApprox(expected[1]));
    solution = tg->getSolution(2);
    ASSERT_TRUE(solution.isApprox(expected[2]));

    tg->solveProblem(TrajectoryGenerator::Solver::BLEIC);
    solution = tg->getSolution(0);
    // Just accept that BLEIC isn't as good
    ASSERT_TRUE(solution.isApprox(expected[0], 1e-6));
    solution = tg->getSolution(1);
    ASSERT_TRUE(solution.isApprox(expected[1], 1e-6));
    solution = tg->getSolution(2);
    ASSERT_TRUE(solution.isApprox(expected[2], 1e-6));

    delete tg;
}

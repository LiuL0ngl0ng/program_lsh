/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: snopt_problem.hpp
 *   Created on: 05, 04, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SNOPT_PROBLEM_HPP
#define OPT_SOLVER_SNOPT_PROBLEM_HPP
#include <snopt/snoptProblem.hpp>
#include "se2_constraints.hpp"
#include "si_bending_energy.hpp"
namespace hmpl {
void usrfun(int *Status, int *n, double x[], int *needF, int *neF, double F[],
            int *needG, int *neG, double G[], char *cu, int *lencu, int iu[],
            int *leniu, double ru[], int *lenru) {
    hmpl::ParamKnot knot;
    knot.p0 = 0;
    knot.p1 = x[0];
    knot.p2 = x[1];
    knot.p3 = 0;
    knot.sf = x[2];

    if (*needF > 0) {
        F[0] = SIBendingEnergy::getSIBendingEnergy(knot);  //  Objective row
        F[1] = SE2Constraints::fxf(knot);
        F[2] = SE2Constraints::fyf(knot);
        F[3] = SE2Constraints::ftf(knot);
    }

    if (*needG > 0) {
        G[0] = SIBendingEnergy::dsiedp1(knot);
        G[1] = SIBendingEnergy::dsiedp2(knot);
        G[2] = SIBendingEnergy::dsiedp4(knot);

        G[3] = SE2Constraints::dxdp1(knot);
        G[4] = SE2Constraints::dxdp2(knot);
        G[5] = SE2Constraints::dxdp4(knot);

        G[6] = SE2Constraints::dydp1(knot);
        G[7] = SE2Constraints::dydp2(knot);
        G[8] = SE2Constraints::dydp4(knot);

        G[9] = SE2Constraints::dtdp1(knot);
        G[10] = SE2Constraints::dtdp2(knot);
        G[11] = SE2Constraints::dtdp4(knot);
    }
}
}
#endif  // OPT_SOLVER_SNOPT_PROBLEM_HPP

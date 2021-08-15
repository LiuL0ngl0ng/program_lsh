/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: nlopt_problem.cpp
 *   Created on: 05, 06, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#include "opt_solver/nlopt_problem.hpp"
#include <opt_utils/utils.hpp>
#include "opt_solver/se2_constraints.hpp"
#include "opt_solver/si_bending_energy.hpp"

using hmpl::SIBendingEnergy;
using hmpl::SE2Constraints;
using hmpl::ParamKnot;

double bendingEnergy(const std::vector<double> &x, std::vector<double> &grad,
                     void *data) {
    hmpl::ParamKnot knot{0, x[0], x[1], 0, x[2]};
    if (!grad.empty()) {
        grad[0] = SIBendingEnergy::dsiedp1(knot);
        grad[1] = SIBendingEnergy::dsiedp2(knot);
        grad[2] = hmpl::SIBendingEnergy::dsiedp4(knot);
    }
    return SIBendingEnergy::getSIBendingEnergy(knot);
}

double xConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data) {
    curvature_constraint_data *d =
            reinterpret_cast<curvature_constraint_data *>(data);
    ParamKnot knot{d->k_init, x[0], x[1], d->k_end, x[2]};
    if (!grad.empty()) {
        grad[0] = SE2Constraints::dxdp1(knot);
        grad[1] = SE2Constraints::dxdp2(knot);
        grad[2] = SE2Constraints::dxdp4(knot);
    }
    return SE2Constraints::fxf(knot) - d->f_end;
}
double yConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data) {
    curvature_constraint_data *d =
            reinterpret_cast<curvature_constraint_data *>(data);
    ParamKnot knot{d->k_init, x[0], x[1], d->k_end, x[2]};
    if (!grad.empty()) {
        grad[0] = SE2Constraints::dydp1(knot);
        grad[1] = SE2Constraints::dydp2(knot);
        grad[2] = SE2Constraints::dydp4(knot);
    }
    return SE2Constraints::fyf(knot) - d->f_end;
}
double tConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data) {
    curvature_constraint_data *d =
            reinterpret_cast<curvature_constraint_data *>(data);
    ParamKnot knot{d->k_init, x[0], x[1], d->k_end, x[2]};
    if (!grad.empty()) {
        grad[0] = SE2Constraints::dtdp1(knot);
        grad[1] = SE2Constraints::dtdp2(knot);
        grad[2] = SE2Constraints::dtdp4(knot);
    }
    return SE2Constraints::fyf(knot) - d->f_end;
}

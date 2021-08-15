/* Copyright (c) 2016, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: QuadraticCostFunction.cpp
 *   Created on: Sep, 19, 2016
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#include "opt_solver/quadratic_cost_function.hpp"

namespace hmpl {
QuadraticCostFunction::QuadraticCostFunction(State initial_state,
                                             State final_state)
    : initial_state_(initial_state),
      final_state_(final_state),
      xy_weight_(25.0),
      heading_weight_(30.0),
      smoothness_weight_(1.0) {}

QuadraticCostFunction::~QuadraticCostFunction() {}

bool QuadraticCostFunction::Evaluate(double const* const* parameters,
                                     double* residuals,
                                     double** jacobians) const {
    ParamKnot param;
    param.p0 = this->initial_state_.k;
    param.p1 = parameters[0][0];
    param.p2 = parameters[0][1];
    param.sf = parameters[0][2];
    param.p3 = this->final_state_.k;
    residuals[0] = this->xy_weight_ *
                   (SE2Constraints::fxf(param) - this->final_state_.x);
    residuals[1] = this->xy_weight_ *
                   (SE2Constraints::fyf(param) - this->final_state_.y);
    residuals[2] = this->heading_weight_ *
                   (SE2Constraints::ftf(param) - this->final_state_.z);
    residuals[3] = SIBendingEnergy::getSIBendingEnergy(param);
    if (jacobians == NULL) {
        return true;
    }
    if (jacobians != NULL && jacobians[0] != NULL) {
        jacobians[0][0] = this->xy_weight_ * SE2Constraints::dxdp1(param);
        jacobians[0][1] = this->xy_weight_ * SE2Constraints::dxdp2(param);
        jacobians[0][2] = this->xy_weight_ * SE2Constraints::dxdp4(param);
        jacobians[0][3] = this->xy_weight_ * SE2Constraints::dydp1(param);
        jacobians[0][4] = this->xy_weight_ * SE2Constraints::dydp2(param);
        jacobians[0][5] = this->xy_weight_ * SE2Constraints::dydp4(param);

        jacobians[0][6] = this->heading_weight_ * SE2Constraints::dtdp1(param);
        jacobians[0][7] = this->heading_weight_ * SE2Constraints::dtdp2(param);
        jacobians[0][8] = this->heading_weight_ * SE2Constraints::dtdp4(param);

        jacobians[0][9] =
                this->smoothness_weight_ * SIBendingEnergy::dsiedp1(param);
        jacobians[0][10] =
                this->smoothness_weight_ * SIBendingEnergy::dsiedp2(param);
        jacobians[0][11] =
                this->smoothness_weight_ * SIBendingEnergy::dsiedp4(param);
    }
    return true;
}
}  // namespace hmpl
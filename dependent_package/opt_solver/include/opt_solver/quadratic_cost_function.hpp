/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: quadratic_cost_function.hpp
 *   Created on: 05, 06, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_QUATRATIC_COST_FUNCTION_H
#define OPT_SOLVER_QUATRATIC_COST_FUNCTION_H

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <opt_utils/utils.hpp>
#include <vector>
#include "se2_constraints.hpp"
#include "si_bending_energy.hpp"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace hmpl {

/**
 * 4 number of residuals, put in another way:the number of cost function terms
 * 3 the number of parameters for each rediduals
 */
class QuadraticCostFunction : public SizedCostFunction<4, 3> {
 public:
    /*!
     * @brief Contructor
     * @param[in] initial_state The initial state in the ego frame.
     * @param[in] final_state The final state in the ego frame.
     * @return None.
     */
    QuadraticCostFunction(State initial_state, State final_state);
    virtual ~QuadraticCostFunction();
    /*!
     * @brief Evaluation the function value and jacobian.
     * @param[in/out] parameters Parameters to be optimized.
     * @param[in] residuals The cost function value
     * @param[in] jacobians The jocabian of cost functions in terms
     * of parameters.
     * @return true if succeed
     */
    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const;

 private:
    /// The initial state of the vehicle in the ego frame.
    State initial_state_;
    /// The final state of the vehicle in the ego frame.
    State final_state_;
    /// The weight for position terms.
    double xy_weight_;
    /// The weight for the heading term.
    double heading_weight_;
    /// The weight for the smoothness term.
    double smoothness_weight_;
};
}  // namespace hmpl
#endif  // OPT_SOLVER_QUATRATIC_COST_FUNCTION_H

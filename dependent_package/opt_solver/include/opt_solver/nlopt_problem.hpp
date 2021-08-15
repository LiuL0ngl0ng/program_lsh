/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: nlopt_problem.hpp
 *   Created on: 05, 06, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_NLOPT_PROBLEM_HPP
#define OPT_SOLVER_NLOPT_PROBLEM_HPP
#include <vector>

typedef struct {
    double k_init;
    double k_end;
    double f_end;
} curvature_constraint_data;

double bendingEnergy(const std::vector<double> &x, std::vector<double> &grad,
                     void *data);
double xConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data);
double yConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data);
double tConstraint(const std::vector<double> &x, std::vector<double> &grad,
                   void *data);

#endif  // OPT_SOLVER_NLOPT_PROBLEM_HPP

/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: nlp_solver.hpp
 *   Created on: 05, 06, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SOLVER_H
#define OPT_SOLVER_SOLVER_H
#include "ipopt_problem.hpp"
#include "nlopt_problem.hpp"
#include "quadratic_cost_function.hpp"
//#include "snopt_problem.hpp"
#include "solver_type.hpp"

namespace hmpl {
class NLPSolver {
 public:
    NLPSolver(SolverType solver_type = SolverType::CERES, bool debug = false);
    /**
     * Solve the two point boundary value problem with the NLP solver
     * @param start[in] initial state
     * @param end[in] final state
     * @param param[in] intial guess of the parameter block
     * @param lower_bounds[in] lower bounds for the parameter block
     * @param upper_bounds[in] upper bounds for the parameter block
     * @param solution[out] solution
     * @return true if succeed
     */
    bool solve(const State &start, const State &end, double *param,
               const double *lower_bounds, const double *upper_bounds,
               ParamKnot *solution);

    /**
     * The ceres problem formulation is different from ipopt. We seperate the
     * specific problem definition and solver interface
     * @param start
     * @param end
     * @param param
     * @param lower_bounds
     * @param upper_bounds
     * @param solution
     * @return
     */
    bool ceresSolve(const State &start, const State &end, double *param,
                    const double *lower_bounds, const double *upper_bounds,
                    ParamKnot *solution);
    /**
     *
     * @param start
     * @param end
     * @param param
     * @param lower_bounds
     * @param upper_bounds
     * @param solution
     * @return
     */
    bool ipoptSolve(const State &start, const State &end, double *param,
                    const double *lower_bounds, const double *upper_bounds,
                    ParamKnot *solution);
    /**
     *
     * @param start
     * @param end
     * @param param
     * @param lower_bounds
     * @param upper_bounds
     * @param solution
     * @return
     */
//    bool snoptSolve(const State &start, const State &end, double *param,
//                    const double *lower_bounds, const double *upper_bounds,
//                    ParamKnot *solution);
    /**
     *
     * @param start
     * @param end
     * @param param
     * @param lower_bounds
     * @param upper_bounds
     * @param solution
     * @return
     */
    bool nloptSolve(const State &start, const State &end, double *param,
                    const double *lower_bounds, const double *upper_bounds,
                    ParamKnot *solution);
    /**
     * Set the debug flag for the solver. This will output the solver full
     * report.
     * @param flag
     */
    void setDebugFlag(bool flag) { this->debug_ = flag; }
    /**
     * Set the solver type
     * @param solver_type
     */
    void setSolverType(SolverType solver_type) {
        this->solver_type_ = solver_type;
    }
    /**
     * Get the solver type
     * @return The solver Enum class type(CERES, IPOPT)
     */
    SolverType getSolverType() { return this->solver_type_; }

 private:
    bool debug_;
    SolverType solver_type_;
};
}
#endif  // OPT_SOLVER_SOLVER_H

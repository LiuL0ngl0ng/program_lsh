/* Copyright (c) 2016, Yu Zhang, Beijing Institute of Technology,
 * All rights reserved.
 *        Files: CTNLP.h
 *   Created on: July, 31, 2016
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_IPOPT_SOLVER_H
#define OPT_SOLVER_IPOPT_SOLVER_H
#pragma GCC diagnostic push             // Save diagnostic state
#pragma GCC diagnostic ignored "-Wall"  // Ignore warnings here
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <coin/IpTNLP.hpp>
#pragma GCC diagnostic pop  // Restore diagnostic state

#include <fstream>
#include <iterator>
#include <map>
#include <opt_utils/utils.hpp>
#include <vector>
#include "se2_constraints.hpp"
#include "si_bending_energy.hpp"

using Ipopt::TNLP;
using Ipopt::Index;
using Ipopt::Number;
using Ipopt::SolverReturn;
using Ipopt::IpoptData;
using Ipopt::IpoptCalculatedQuantities;
using Ipopt::SmartPtr;
using Ipopt::ApplicationReturnStatus;
using Ipopt::Solve_Succeeded;
using hmpl::State;
using hmpl::ParamKnot;
using hmpl::ParamCoefficient;
/** Constrained optimization path generation for interfacing a problem with
 * IPOPT.
 *  CTNLP implements a path generator showing how to interface with IPOPT
 *  through the TNLP interface.
 */

namespace hmpl {
class CTNLP : public TNLP {
 public:
    /**
     * Constructor for the problem
     * @param initial_state
     * @param final_state
     * @param initial_guess
     * @param k_l
     * @param k_u
     * @param len_l
     * @param len_u
     */
    CTNLP(State initial_state, State final_state, ParamKnot initial_guess,
          double k_l, double k_u, double len_l, double len_u);

    /** default destructor */
    virtual ~CTNLP();

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                              Index& nnz_h_lag, IndexStyleEnum& index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                                 Number* g_l, Number* g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Index m, bool init_lambda, Number* lambda);

    /** Method to return the objective value */
    virtual bool eval_f(Index n, const Number* x, bool new_x,
                        Number& obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x,
                             Number* grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m,
                        Number* g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                            Index nele_jac, Index* iRow, Index* jCol,
                            Number* values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not
     * NULL)
     */
    virtual bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                        Index m, const Number* lambda, bool new_lambda,
                        Index nele_hess, Index* iRow, Index* jCol,
                        Number* values);

    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete so the TNLP can
     * store/write the solution */
    virtual void finalize_solution(SolverReturn status, Index n,
                                   const Number* x, const Number* z_L,
                                   const Number* z_U, Index m, const Number* g,
                                   const Number* lambda, Number obj_value,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq);
    //@}

 private:
    /**@name Methods to block default compiler methods.
     * The compiler automatically generates the following three methods.
     *  Since the default compiler implementation is generally not what
     *  you want (for all but the most simple classes), we usually
     *  put the declarations of these methods in the private section
     *  and never implement them. This prevents the compiler from
     *  implementing an incorrect "default" behavior without us
     *  knowing. (See Scott Meyers book, "Effective C++")
     *
     */
    //@{
    //  CTNLP();
    CTNLP(const CTNLP&);
    CTNLP& operator=(const CTNLP&);

 private:
    State startState_;
    State endState_;
    double k_l_;       // lower boundary for curvature
    double k_u_;       // upper boundary for curvature
    double length_l_;  // lower boundary for arc-length
    double length_u_;  // upper boundary for arc-length

    ParamKnot initial_guess_;         // initial guess for parameters
    std::vector<State> path_vector_;  // store the path state
    // store the solution of the optimizer
    ParamKnot solution_;

 public:
    ParamKnot get_solution(void) { return solution_; }
    State get_startState(void) { return startState_; }
    State get_endState(void) { return endState_; }
};

}  // namespace hmpl
#endif  // OPT_SOLVER_IPOPT_SOLVER_H

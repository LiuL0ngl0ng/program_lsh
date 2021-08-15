#include "opt_solver/ipopt_problem.hpp"
#include <cassert>
#include <opt_utils/constants.hpp>
using hmpl::SIBendingEnergy;
using hmpl::SE2Constraints;
using hmpl::SAMPLE_NUM;
/* Constructor. */

namespace hmpl {

/*!
 * Constructor with inputs, pass key information of TVBVP to this class.
 * @param startState [in] start state of the vehicles, including x y heading and
 * curvature
 * @param finalState [in] final state of the vehicles
 * @param initialGuess [in] initial guess of knot parameter
 * @param kLower [in] lower boundary for curvature
 * @param kUpper [in] upper boundary for curvature
 * @param lenLower [in] lower boundary for the arc-length of the curve
 * @param lenUpper [in] upper boundary for the arc-length of the curve
 * @param resolution [in] this is reserved for resolution of the map
 * @param mode [in] reserved
 * @return none
 */
CTNLP::CTNLP(State startState, State finalState, ParamKnot initialGuess,
             double kLower, double kUpper, double lenLower, double lenUpper)
    : startState_(startState),
      endState_(finalState),
      k_l_(kLower),
      k_u_(kUpper),
      length_l_(lenLower),
      length_u_(lenUpper),
      initial_guess_(initialGuess) {
    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
}

CTNLP::~CTNLP() {}

/* -------------------------------------- */
/**
 * @Synopsis Give IPOPT the information about the size of the problem(and
 * hendce, the size of the arrays that it needs to allocate.)
 *
 * @Param n [out], the number of variables in the problem(dimension of x)
 * @Param m [out], the number of constraints in the problem(dimension of g(x))
 * @Param nnz_jac_g [out], the number of nonzero entries in the Jacobian.
 * @Param nnz_h_lag [out], the number of nonzero entries in the Hessian
 * @Param index_style [out], the numbering style used for row/col entries in the
 * sparse matrix format
 *
 * @Returns true
 */
/* -------------------------------------- */
bool CTNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                         IndexStyleEnum& index_style) {
    // three variables
    n = 3;
    // three equality constraint,
    m = 3;
    nnz_jac_g = n * m;
    // and 9 nonzeros in the hessian of the lagrangian
    // symmetric , so it becomes 6
    nnz_h_lag = 6;  //  one objective, three variables
    index_style =
            TNLP::C_STYLE;  // fortran_style is 1 based, C_STYLE is 0 based
    return true;
}

/**
 * Pass the bounds information to ipopt
 * @param n [in] number of variables
 * @param x_l [out] lower boundary for variables
 * @param x_u [out] upper boundary for variables
 * @param m [in] number of constraints
 * @param g_l [out] lower boundary for constraints
 * @param g_u [out] upper boundary for constraints
 * @return succeed
 */
bool CTNLP::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                            Number* g_l, Number* g_u) {
    //  for p1  the 1/3 of the path
    x_l[0] = k_l_;
    x_u[0] = k_u_;

    // for p2 the 2/3 of the path
    x_l[1] = k_l_;
    x_u[1] = k_u_;

    // for p3 the end of the path
    x_l[2] = length_l_;
    x_u[2] = length_u_;

    // we have 3 equality constraints, so we set the bounds on these constraints
    // to be equal.
    g_l[0] = g_u[0] = endState_.x;
    g_l[1] = g_u[1] = endState_.y;
    g_l[2] = g_u[2] = endState_.z;

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)m;
    return true;
}
/**
 * Pass the start point information to Ipopt
 * @param n [in] number of variables
 * @param init_x [in] not used
 * @param x [out] initial guess for parameters
 * @param init_z not used
 * @param z_L not used
 * @param z_U not used
 * @param m not used
 * @param init_lambda not used
 * @param lambda not used
 * @return Succeed
 */
bool CTNLP::get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                               Number* z_L, Number* z_U, Index m,
                               bool init_lambda, Number* lambda) {
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the others if
    // you wish.
    x[0] = initial_guess_.p1;
    x[1] = initial_guess_.p2;
    x[2] = initial_guess_.sf;

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)init_x;
    (void)init_z;
    (void)z_L;
    (void)z_U;
    (void)m;
    (void)init_lambda;
    (void)lambda;
    return true;
}
/*!
 * Evaluate the value of objective function
 * @param n not using
 * @param x [in] updated parameters from ipopt
 * @param new_x  not using
 * @param obj_value [out] value of objective functions
 * @return secceed
 */
bool CTNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
    // return the value of the objective function
    ParamKnot param;
    param.p0 = startState_.k;
    param.p1 = x[0];
    param.p2 = x[1];
    param.p3 = endState_.k;
    param.sf = x[2];

    obj_value = SIBendingEnergy::getSIBendingEnergy(param);

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)new_x;
    return true;
}

/*!
 * Evaluate gradient of objective functions
 * @param n not used.
 * @param x [in] updated parameters from ipopt engine
 * @param new_x not used
 * @param grad_f [out] gradient array of objective functions in term of new
 * parameters
 * @return succeed
 */
bool CTNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
    // return the gradient of the objective function grad_{x} f(x)
    ParamKnot param;
    param.p0 = startState_.k;
    param.p1 = x[0];
    param.p2 = x[1];
    param.p3 = endState_.k;
    param.sf = x[2];

    grad_f[0] = SIBendingEnergy::dsiedp1(param);
    grad_f[1] = SIBendingEnergy::dsiedp2(param);
    grad_f[2] = SIBendingEnergy::dsiedp4(param);

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)new_x;
    return true;
}

/*!
 * Evaluate constraints functions
 * @param n [in] number of variables
 * @param x [in] updated parameters from ipopt engine
 * @param new_x not used
 * @param m [in] number of constraints
 * @param g [out] not used WRONG - TODO(yuzhangbit)
 * @return succeed
 */
bool CTNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
    // there are three constraints here.
    ParamKnot param;
    param.p0 = startState_.k;
    param.p1 = x[0];
    param.p2 = x[1];
    param.p3 = endState_.k;
    param.sf = x[2];

    g[0] = SE2Constraints::fxf(param);
    g[1] = SE2Constraints::fyf(param);
    g[2] = SE2Constraints::ftf(param);
    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)new_x;
    (void)m;
    return true;
}

/*!
 * Evaluate Jacobian of constraints functions in terms of variables
 * @param n [in] number of parameters
 * @param x [in] updated parameters from ipopt engine
 * @param new_x not used
 * @param m [in] number of constraints
 * @param nele_jac not used
 * @param iRow [in] rows of jacobian(format jacobian matrix)
 * @param jCol [in] collums of jacobian
 * @param values [in] values elements of jacobian matrix
 * @return succeed
 */
bool CTNLP::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                       Index nele_jac, Index* iRow, Index* jCol,
                       Number* values) {
    if (values == NULL) {
        Index idx = 0;
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                iRow[idx] = i;
                jCol[idx] = j;
                idx++;
            }
        }
    } else {
        ParamKnot param;
        param.p0 = startState_.k;
        param.p1 = x[0];
        param.p2 = x[1];
        param.p3 = endState_.k;
        param.sf = x[2];

        values[0] = SE2Constraints::dxdp1(param);
        values[1] = SE2Constraints::dxdp2(param);
        values[2] = SE2Constraints::dxdp4(param);

        values[3] = SE2Constraints::dydp1(param);
        values[4] = SE2Constraints::dydp2(param);
        values[5] = SE2Constraints::dydp4(param);

        values[6] = SE2Constraints::dtdp1(param);
        values[7] = SE2Constraints::dtdp2(param);
        values[8] = SE2Constraints::dtdp4(param);
    }

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)new_x;
    (void)nele_jac;
    return true;
}

/*!
 * Evaluate hassian of the objective functions and constraints functions.
 * @param n [out] numbers of parameters
 * @param x [out] updated parameters from ipopt engine
 * @param new_x not used
 * @param obj_factor [out] coefficient of objective functions in hessian
 * @param m [out] number of constraints
 * @param lambda [out] coefficient of constraints functions in hessian
 * @param new_lambda not used
 * @param nele_hess not used
 * @param iRow [in] rows of hessian
 * @param jCol [in] collums of hessian
 * @param values [in] values of elements in hessian
 * @return succeed
 */
bool CTNLP::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                   Index m, const Number* lambda, bool new_lambda,
                   Index nele_hess, Index* iRow, Index* jCol, Number* values) {
    if (values == NULL) {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.
        Index idx = 0;
        for (Index row = 0; row < 3; row++) {
            for (Index col = 0; col <= row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }

        assert(idx == nele_hess);
    } else {
        // return the values
        ParamKnot param;
        param.p0 = startState_.k;
        param.p1 = x[0];
        param.p2 = x[1];
        param.p3 = endState_.k;
        param.sf = x[2];

        values[0] = SIBendingEnergy::dsied11(param) * obj_factor;
        values[1] = SIBendingEnergy::dsied21(param) * obj_factor;
        values[2] = SIBendingEnergy::dsied22(param) * obj_factor;
        values[3] = SIBendingEnergy::dsied41(param) * obj_factor;
        values[4] = SIBendingEnergy::dsied42(param) * obj_factor;
        values[5] = SIBendingEnergy::dsied44(param) * obj_factor;

        values[0] += lambda[0] * SE2Constraints::dxd11(param);
        values[1] += lambda[0] * SE2Constraints::dxd21(param);
        values[2] += lambda[0] * SE2Constraints::dxd22(param);
        values[3] += lambda[0] * SE2Constraints::dxd41(param);
        values[4] += lambda[0] * SE2Constraints::dxd42(param);
        values[5] += lambda[0] * SE2Constraints::dxd44(param);

        values[0] += lambda[1] * SE2Constraints::dyd11(param);
        values[1] += lambda[1] * SE2Constraints::dyd21(param);
        values[2] += lambda[1] * SE2Constraints::dyd22(param);
        values[3] += lambda[1] * SE2Constraints::dyd41(param);
        values[4] += lambda[1] * SE2Constraints::dyd42(param);
        values[5] += lambda[1] * SE2Constraints::dyd44(param);

        values[0] += lambda[2] * SE2Constraints::dtd11(param);
        values[1] += lambda[2] * SE2Constraints::dtd21(param);
        values[2] += lambda[2] * SE2Constraints::dtd22(param);
        values[3] += lambda[2] * SE2Constraints::dtd41(param);
        values[4] += lambda[2] * SE2Constraints::dtd42(param);
        values[5] += lambda[2] * SE2Constraints::dtd44(param);
    }

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)n;
    (void)new_x;
    (void)m;
    (void)new_lambda;
    return true;
}

/*!
 * Get the final solution from ipopt and save them.
 * @param status [out] status of solver
 * @param n [out] number of parameters
 * @param x [out] values of parameters
 * @param z_L not used
 * @param z_U not used
 * @param m [out] numbers of constraints
 * @param g not used
 * @param lambda not used
 * @param obj_value [out] final value of objective functions
 * @param ip_data  [out] internal ipopt data, not use
 * @param ip_cq not used
 */
void CTNLP::finalize_solution(SolverReturn status, Index n, const Number* x,
                              const Number* z_L, const Number* z_U, Index m,
                              const Number* g, const Number* lambda,
                              Number obj_value, const IpoptData* ip_data,
                              IpoptCalculatedQuantities* ip_cq) {
    solution_.p0 = startState_.k;
    solution_.p1 = x[0];
    solution_.p2 = x[1];
    solution_.p3 = endState_.k;
    solution_.sf = x[2];

    // Don't need all of IpOpt's parameters, suppress -Wunused-parameter
    (void)status;
    (void)n;          // the number of the decision variables
    (void)z_L;        // the final values for the lower bound multipliers
    (void)z_U;        // the final values for the upper bound multipliers
    (void)m;          //  the number of constraints in the problem
    (void)g;          // the final value of the constraint function values
    (void)lambda;     // the final values of the constraint multipliers
    (void)obj_value;  // the final value of the objective
    (void)ip_data;
    (void)ip_cq;
}

}  // namespace hmpl

// ipopt
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
// nlopt
#include <nlopt.hpp>
#include "opt_solver/nlp_solver.hpp"

namespace hmpl {

NLPSolver::NLPSolver(SolverType solver_type, bool debug)
    : debug_(debug), solver_type_(solver_type) {}

bool NLPSolver::solve(const State &start, const State &end, double *param,
                      const double *lower_bounds, const double *upper_bounds,
                      ParamKnot *solution) {
    if (debug_) {
        std::cout << "Start:" << start.x << "," << start.y << "," << start.z
                  << "," << start.k << std::endl;
        std::cout << "End:" << end.x << "," << end.y << "," << end.z << ","
                  << end.k << std::endl;
    }
    switch (this->solver_type_) {
        case SolverType::CERES:
            return ceresSolve(start, end, param, lower_bounds, upper_bounds,
                              solution);
            break;  // not sure about this, should i keep it?
        case SolverType::IPOPT:
            return ipoptSolve(start, end, param, lower_bounds, upper_bounds,
                              solution);
            break;  // not sure about this, should i keep it?
        case SolverType::SNOPT:
//            return snoptSolve(start, end, param, lower_bounds, upper_bounds,
//                              solution);
            break;

        case SolverType::NLOPT:
            return nloptSolve(start, end, param, lower_bounds, upper_bounds,
                              solution);
            break;
        default:
            std::cout << "The solver type is not supported for now."
                      << std::endl;
            return false;
            break;
    }
}
bool NLPSolver::ceresSolve(const State &start, const State &end, double *param,
                           const double *lower_bounds,
                           const double *upper_bounds, ParamKnot *solution) {
    Problem problem;
    CostFunction *cost = new QuadraticCostFunction(start, end);
    problem.AddResidualBlock(cost, NULL, param);
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_QR;
    for (int i = 0; i < cost->parameter_block_sizes().front(); i++) {
        problem.SetParameterLowerBound(param, i, lower_bounds[i]);
        problem.SetParameterUpperBound(param, i, upper_bounds[i]);
    }

    ceres::Solve(options, &problem, &summary);
    if (debug_) {
        std::cout << summary.FullReport() << std::endl;
    }
    if (summary.IsSolutionUsable()) {
        solution->p0 = start.k;
        solution->p1 = param[0];
        solution->p2 = param[1];
        solution->p3 = end.k;
        solution->sf = param[2];
        return true;
    } else {
        return false;
    }
}
bool NLPSolver::ipoptSolve(const State &start, const State &end, double *param,
                           const double *lower_bounds,
                           const double *upper_bounds, ParamKnot *solution) {
    SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    ApplicationReturnStatus status;  // app return status
    // set up the options for the solver
    app->RethrowNonIpoptException(true);
    app->Options()->SetNumericValue("tol", 1e-6);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetStringValue("nlp_scaling_method", "gradient-based");
    app->Options()->SetStringValue("derivative_test", "second-order");
    app->Options()->SetIntegerValue("max_iter", 50);
    app->Initialize();
    ParamKnot initial_guess;
    initial_guess.p0 = start.k;
    initial_guess.p1 = param[0];
    initial_guess.p2 = param[1];
    initial_guess.p3 = end.k;
    initial_guess.sf = param[2];
    SmartPtr<hmpl::CTNLP> nlp =
            new CTNLP(start, end, initial_guess, lower_bounds[0],
                      upper_bounds[0], lower_bounds[2], upper_bounds[2]);

    status = app->OptimizeTNLP(nlp);

    if (status == Solve_Succeeded) {
        *solution = GetRawPtr(nlp)->get_solution();
        return true;
    } else {
        return false;
    }
}
//bool NLPSolver::snoptSolve(const State &start, const State &end, double *param,
//                           const double *lower_bounds,
//                           const double *upper_bounds, ParamKnot *solution) {
//    snoptProblemA nlp;
//    // Allocate and initialize;
//    int neF = 4;
//    int n = 3;
//
//    int nS = 0, nInf;
//    double sInf;
//
//    double *x = new double[n];
//    double *xlow = new double[n];
//    double *xupp = new double[n];
//    double *xmul = new double[n];
//    int *xstate = new int[n];
//
//    double *F = new double[neF];
//    double *Flow = new double[neF];
//    double *Fupp = new double[neF];
//    double *Fmul = new double[neF];
//    int *Fstate = new int[neF];
//
//    int ObjRow = 0;
//    double ObjAdd = 0;
//
//    int Cold = 0, Basis = 1, Warm = 2;
//
//    int lenG = 12;
//    int *iGfun = new int[lenG];
//    int *jGvar = new int[lenG];
//    int neG = lenG;
//    // Set the upper and lower bounds.
//    for (int i = 0; i < n; i++) {
//        xlow[i] = lower_bounds[i];
//        xupp[i] = upper_bounds[i];
//    }
//
//    x[0] = param[0];
//    x[1] = param[1];
//    x[2] = param[2];
//    xstate[0] = 0;
//    xstate[1] = 0;
//    xstate[2] = 0;
//
//    Flow[0] = -1e20;
//    Flow[1] = end.x;
//    Flow[2] = end.y;
//    Flow[3] = end.z;
//    Fupp[0] = 1e20;
//    Fupp[1] = end.x;
//    Fupp[2] = end.y;
//    Fupp[3] = end.z;
//    Fmul[0] = 0;
//    Fmul[1] = 0;
//    Fmul[2] = 0;
//    Fmul[3] = 0;
//
//    // snopta will compute the Jacobian by finite-differences.
//    // snJac will be called  to define the
//    // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
//    // Provide the elements of the Jacobian explicitly.
//    iGfun[0] = 0;
//    jGvar[0] = 0;
//
//    iGfun[1] = 0;
//    jGvar[1] = 1;
//
//    iGfun[2] = 0;
//    jGvar[2] = 2;
//
//    iGfun[3] = 1;
//    jGvar[3] = 0;
//
//    iGfun[4] = 1;
//    jGvar[4] = 1;
//
//    iGfun[5] = 1;
//    jGvar[5] = 2;
//
//    iGfun[6] = 2;
//    jGvar[6] = 0;
//
//    iGfun[7] = 2;
//    jGvar[7] = 1;
//
//    iGfun[8] = 2;
//    jGvar[8] = 2;
//
//    iGfun[9] = 3;
//    jGvar[9] = 0;
//
//    iGfun[10] = 3;
//    jGvar[10] = 1;
//
//    iGfun[11] = 3;
//    jGvar[11] = 2;
//
//    int lenA = 1;
//    int *iAfun = new int[lenA];
//    int *jAvar = new int[lenA];
//    double *A = new double[lenA];
//    int neA = 0;
//
//    // Load the data for ToyProb ...
//    nlp.initialize("", 0);  // no print file; summary on
//    nlp.setIntParameter("Derivative option", 1);
//    nlp.setIntParameter("Verify level ", 3);
//    nlp.setProbName("NLP");
//    nlp.setIntParameter("Major Iteration limit", 100);
//    nlp.setIntParameter("Verify level ", 3);
//    nlp.solve(Cold, neF, n, ObjAdd, ObjRow, usrfun, iAfun, jAvar, A, neA, iGfun,
//              jGvar, neG, xlow, xupp, Flow, Fupp, x, xstate, xmul, F, Fstate,
//              Fmul, nS, nInf, sInf);
//
//    solution->p0 = 0;
//    solution->p1 = x[0];
//    solution->p2 = x[1];
//    solution->p3 = 0;
//    solution->sf = x[2];
//
//    delete[] iAfun;
//    delete[] jAvar;
//    delete[] A;
//    delete[] iGfun;
//    delete[] jGvar;
//
//    delete[] x;
//    delete[] xlow;
//    delete[] xupp;
//    delete[] xmul;
//    delete[] xstate;
//
//    delete[] F;
//    delete[] Flow;
//    delete[] Fupp;
//    delete[] Fmul;
//    delete[] Fstate;
//
//    return true;
//}
bool NLPSolver::nloptSolve(const State &start, const State &end, double *param,
                           const double *lower_bounds,
                           const double *upper_bounds, ParamKnot *solution) {
    /*nlopt::opt opt(nlopt::AUGLAG_EQ, 3);
    nlopt::opt local_opt(nlopt::LD_MMA, 3);
    opt.set_local_optimizer(local_opt);
    opt.set_maxtime(0.1);
    // nlopt::opt local_opt(nlopt::LD_SLSQP, 3);
    // opt.set_local_optimizer(local_opt);
    std::vector<double> lb;
    std::vector<double> ub;
    for (int i = 0; i < 3; i++) {
        lb.push_back(lower_bounds[i]);
        ub.push_back(upper_bounds[i]);
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    curvature_constraint_data kdata = {start.k, end.k, 0};
    opt.set_min_objective(bendingEnergy, &kdata);

    curvature_constraint_data xdata = {start.k, end.k, end.x};
    curvature_constraint_data ydata = {start.k, end.k, end.y};
    curvature_constraint_data tdata = {start.k, end.k, end.z};

    double tolenrance = 1e-5;
    opt.add_equality_constraint(xConstraint, &xdata, tolenrance);
    opt.add_equality_constraint(yConstraint, &ydata, tolenrance);
    opt.add_equality_constraint(tConstraint, &tdata, tolenrance);

    std::vector<double> opt_param;
    for (int i = 0; i < opt.get_dimension(); i++) {
        opt_param.push_back(param[i]);
    }

    double f = 0;
    std::cout << "before nlopt optimize." << std::endl;
    nlopt::result res = opt.optimize(opt_param, f);
    std::cout << "after nlopt optimization." << std::endl;

    if (res > 0) {
        solution->p0 = start.k;
        solution->p1 = opt_param[0];
        solution->p2 = opt_param[1];
        solution->p3 = end.k;
        solution->sf = opt_param[2];
        std::cout << "nlopt optimization done." << std::endl;
        return true;
    } else {
        std::cout << "nlopt optimization failed." << std::endl;
        return false;
    }*/
    return false;
}

}  // namespace hmpl
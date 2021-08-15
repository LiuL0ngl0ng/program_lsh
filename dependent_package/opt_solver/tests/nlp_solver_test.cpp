#include "opt_solver/nlp_solver.hpp"
#include <gtest/gtest.h>
#include <chrono>

using hmpl::State;
using hmpl::ParamKnot;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(NLPSolverTest, solve) {
    hmpl::NLPSolver solver;
    State start{}, end;
    end.x = 10;
    end.y = 2;
    end.z = 1.5;
    end.k = 0;
    end.s = 11;
    end.v = 0;

    const double lower_bounds[3] = {-1.0, -1.0, 10.0};
    const double upper_bounds[3] = {1.0, 1.0, 100.0};

    double param[3] = {0, 0, 11};
    ParamKnot solution;

    EXPECT_TRUE(solver.solve(start, end, param, lower_bounds, upper_bounds,
                             &solution));
    std::cout << "Solution:" << solution.p0 << "," << solution.p1 << ","
              << solution.p2 << "," << solution.p3 << std::endl;
}
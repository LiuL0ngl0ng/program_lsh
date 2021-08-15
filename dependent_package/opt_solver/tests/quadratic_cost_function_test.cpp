//
// Created by ragdoll on 01/05/17.
//
#include "opt_solver/quadratic_cost_function.hpp"
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(QuadraticCostFunctionTest, constructor) {
    hmpl::State start, end;
    end.x = 10;
    end.y = 1;
    end.z = 0;
    end.k = 0;
    end.s = 11;
    end.v = 0;
    hmpl::QuadraticCostFunction cost(start, end);
    std::cout << cost.parameter_block_sizes().front() << std::endl;
}
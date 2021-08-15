/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: car_geometry_test.cpp
 *   Created on: 05, 12, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include <car_model/car_geometry.hpp>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(CarGeometryTest, constructor) {
    hmpl::CarGeometry car;
}

TEST(CarGeometryTest, boundingCircle) {
    hmpl::CarGeometry car;
    EXPECT_DOUBLE_EQ(1.36025, car.getBoundingCircle().position.x);
    EXPECT_DOUBLE_EQ(0, car.getBoundingCircle().position.y);
    EXPECT_DOUBLE_EQ(2.739406881516508, car.getBoundingCircle().r);
}

/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: circle.hpp
 *   Created on: 28, 09, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_CIRCLES_HPP
#define OPT_UTILS_CIRCLES_HPP

#include <vector>

#include "vector2d.hpp"

namespace hmpl {

class Circle {
 public:
    // the circle position
    Vector2D<double> position;

    // the circle radius
    double r;

    // simplest constructor
    Circle();

    // basic constructor
    Circle(const Vector2D<double>&, double);

    // most explicit constructor
    Circle(double x, double y, double radius);

    // copy constructor
    Circle(const Circle&);
};

// define the general circle references and pointers
typedef Circle* CirclePtr;
typedef Circle& CircleRef;

class CircleArray {
 public:
    // the circle list
    std::vector<Circle> circles;
};

typedef CircleArray* CircleArrayPtr;
}  // hmpl

#endif  // OPT_UTILS_CIRCLES_HPP

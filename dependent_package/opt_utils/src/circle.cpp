#include "opt_utils/circle.hpp"

using namespace hmpl;
// simplest constructor
Circle::Circle() : position(0.0, 0.0), r(0.0) {}

// basic constructor
Circle::Circle(const Vector2D<double> &p, double _r) : position(p), r(_r) {}

// most explicit constructor
Circle::Circle(double x, double y, double radius) : position(x, y), r(radius) {}

// copy constructor
Circle::Circle(const Circle &c) : position(c.position), r(c.r) {}

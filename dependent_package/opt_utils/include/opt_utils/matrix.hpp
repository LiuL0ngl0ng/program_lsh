/* Copyright (c) 2017, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: matrix.hpp
 *   Created on: Sep, 28, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_MATRIX_TEMPLATE_HPP
#define OPT_UTILS_MATRIX_TEMPLATE_HPP

#include <array>

namespace hmpl {

template <typename T, unsigned int rows, unsigned int cols>
class MatrixT {
 private:
    // place holder
 public:
    // PUBLIC ATTRIBUTES

    // the actual matrix
    std::array<std::array<T, cols>, rows> m;

    // PUBLIC METHODS

    // basic constructor
    MatrixT() {}
};
}  // namespace hmpl

#endif  // OPT_UTILS_MATRIX_TEMPLATE_HPP

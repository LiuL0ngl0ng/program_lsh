//
// Created by yangt on 19-4-22.
//

#ifndef VORONOI_VORONOI_DIAGRAM_IN_HPP
#define VORONOI_VORONOI_DIAGRAM_IN_HPP

template<class cvMatType>
void VoronoiDiagram::reduceScaleCV(const cvMatType &intput_matrix,
                                   int scale,
                                   cvMatType *output_mat) {
    cv::resize(intput_matrix,
               *output_mat,
               cv::Size(intput_matrix.rows / scale, intput_matrix.cols / scale));
}

template<class BasicType>
void VoronoiDiagram::reduceScale(cv::Mat_<BasicType> &matrix,
                                 int scale,
                                 cv::Mat_<BasicType> *mat) {
// the reduced mat size should be : (original_size - (scale + 1) / 2) / 2 + 1
    // for example, scale being 3, then
    // original size          reduced mat size
    // 2 or 3 or 4            1
    // 5 or 6 or 7            2
    // 8 or 9 or 10           3
    // ...                    ...
    int center_num = (scale + 1) / 2;
    mat->create((matrix.rows - center_num) / scale + 1, (matrix.cols - center_num) / scale + 1);
    for (int i = scale / 2; i < matrix.rows; i += scale) {
        for (int j = scale / 2; j < matrix.cols; j += scale) {
            auto value = matrix(i, j);
            (*mat)(i / scale, j / scale) = value;
            if (empty_flag && value == OCCUPANCY) {
                empty_flag = false;
            }
        }
    }
}

template<class cvMatType>
void VoronoiDiagram::optimizeInputNew(const cvMatType &input,
                                      cvMatType *output) {
    cv::Mat kernal1 =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
    cv::morphologyEx(input, *output, cv::MORPH_OPEN, kernal1);
}


template<class cvMatType>
void VoronoiDiagram::expandReducedVoronoi(const cvMatType &reduced_voronoi,
                                 int scale,
                                 cvMatType *expanded_voronoi) {
    for (int i = 0; i != reduced_voronoi.rows; ++i) {
        for (int j = 0; j != reduced_voronoi.cols; ++j) {
            if (reduced_voronoi(i, j) == FREE) {
                int expanded_row = i * scale + 1;
                int expanded_col = j * scale + 1;
                (*expanded_voronoi)(expanded_row, expanded_col) = FREE;
                // Connect the adjacent edge points so that the edges won't be discrete in the voronoi diagram of original size.
                if (j + 1 < expanded_voronoi->cols / scale && reduced_voronoi(i, j + 1) == FREE) {
                    for (int k = 1; k < scale; ++k) {
                        (*expanded_voronoi)(expanded_row, expanded_col + k) = OCCUPANCY;
                    }
                }
                if (i + 1 < expanded_voronoi->rows / scale && j - 1 > 0 && reduced_voronoi(i + 1, j - 1) == FREE) {
                    for (int k = 1; k < scale; ++k) {
                        (*expanded_voronoi)(expanded_row + k, expanded_col - k) = OCCUPANCY;
                    }
                }
                if (i + 1 < expanded_voronoi->rows / scale && reduced_voronoi(i + 1, j) == FREE) {
                    for (int k = 1; k < scale; ++k) {
                        (*expanded_voronoi)(expanded_row + k, expanded_col) = OCCUPANCY;
                    }
                }
                if (i + 1 < expanded_voronoi->rows / scale && j + 1 < expanded_voronoi->cols / scale && reduced_voronoi(i + 1, j + 1) == FREE) {
                    for (int k = 1; k < scale; ++k) {
                        (*expanded_voronoi)(expanded_row + k, expanded_col + k) = OCCUPANCY;
                    }
                }
            }
        }
    }
}

#endif //VORONOI_VORONOI_DIAGRAM_IN_HPP

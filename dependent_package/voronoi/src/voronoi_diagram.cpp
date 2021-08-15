//
// Created by lijiangnan on 19-4-14.
//
#include "voronoi/voronoi_diagram.hpp"
#include <glog/logging.h>
VoronoiDiagram::VoronoiDiagram()
    : is_valid_(false) {

}

VoronoiDiagram::VoronoiDiagram(const grid_map::GridMap &map,
                               int scale,
                               bool display)
    : is_valid_(false),
      scale_(scale),
      resolution_(map.getResolution()),
      voronoi_distance_({"distance"}),
      map_rows_(map.get("obstacle").rows()),
      map_cols_(map.get("obstacle").cols()),
      empty_flag(true){
    CHECK(map.exists("obstacle"))
    << "input grid map required a layer named obstacle!";
    cv::Mat obstacle_mat;
    cv::eigen2cv(map.get("obstacle"), obstacle_mat);
    cv::Mat1b input_mat;
    obstacle_mat.convertTo(input_mat, CV_8UC1);
    // optimize input
    optimizeInputNew(input_mat, &input_mat);

    cv::Mat1b reduced_mat;
    reduceScale(input_mat, scale, &reduced_mat);
//    reduceScaleCV(input_mat,
//                  scale,
//                  &reduced_mat);

    VoronoiThinner voronoi_generator;
    bool ok = false;
    if (empty_flag) {
        LOG(INFO) << "No Obstalces. Abondon Voronoi Diagram!";
        return;
    } else {
        ok = voronoi_generator.thin(reduced_mat, "morph", false);
    }

    if (ok) {
        this->voronoi_ = voronoi_generator.get_skeleton();
        if (voronoi_.rows != reduced_mat.rows
            || voronoi_.cols != reduced_mat.cols) {
            LOG(WARNING) << "Voronoi Diagram Generation Failed!";
            return;
        }
        this->map_center_ = map.getPosition();
        voronoi_distance_.setGeometry(grid_map::Length(
            map.get("obstacle").cols() * resolution_, map.get("obstacle").rows() * resolution_),
                                      resolution_);

        // Convert voronoi diagram of reduced size to a mat with original size. The quality of voronoi diagram is improved
        // because the distance to edges are more accurate.
        // Create a mat of original size.
//        expanded_voronoi_.create(map_rows_, map_cols_);
        cv::Mat1b expanded_voronoi = cv::Mat1b::ones(map_rows_, map_cols_);
        expandReducedVoronoi(voronoi_, scale, &expanded_voronoi);
        cv::Mat obs_f = hmpl::eigen2cv(voronoi_distance_.get("distance"));
        cv::distanceTransform(expanded_voronoi,
                              obs_f,
                              CV_DIST_L2,
                              CV_DIST_MASK_PRECISE);
        this->is_valid_ = true;
        if (display) {
            cv::imshow("voronoi", voronoi_);
            cv::waitKey(1);
        }
    } else {
        LOG(WARNING) << "Voronoi Diagram Generation Failed!";
        return;
    }
}

double VoronoiDiagram::getPotential(double dis_to_obs,
                                    const grid_map::Position &pt) const {
    double potential = 0.0;
    if (dis_to_obs < this->distance_threshold_) {
        const double dis_to_edge = getMinDistanceToEdgesNew(pt(0), pt(1));
        // +1: potential on voronoi edges is no longer 0, it's also related to the distance to obstacles.
        potential = (dis_to_edge + 1)
            / (dis_to_edge + dis_to_obs + 1)
            * fabs(dis_to_obs - distance_threshold_) / distance_threshold_;
    }
    return potential;
}

double VoronoiDiagram::getPotential(double dis_to_obs,
                                    const grid_map::Index &index) const {
    double potential = 0.0;
    if (dis_to_obs < this->distance_threshold_) {
        const double dis_to_edge = getMinDistanceToEdgesNew(index(0), index(1));
        // +1: potential on voronoi edges is no longer 0, it's also related to the distance to obstacles.
        potential = (dis_to_edge + 1)
            / (dis_to_edge + dis_to_obs + 1)
            * fabs(dis_to_obs - distance_threshold_) / distance_threshold_;
    }
    return potential;
}

double VoronoiDiagram::getPotential(const grid_map::GridMap &map,
                                    const grid_map::Index &index) const {
    CHECK(map.exists("distance"))
    << "getPotential require input grid map has a layer named distance!";

    const double dis_to_obs = map.at("distance", index);
    return getPotential(dis_to_obs, index);
}

double VoronoiDiagram::getPotential(const grid_map::GridMap &map,
                                    const grid_map::Position &pt) const {
    CHECK(map.exists("distance"))
    << "getPotential require input grid map has a layer named distance!";

    const auto &dis_to_obs = map.atPosition("distance",
                                            pt,
                                            grid_map::InterpolationMethods::INTER_LINEAR);
    return getPotential(dis_to_obs, pt);
}

double VoronoiDiagram::getMinDistanceToEdgesNew(int row, int col) const {
//    grid_map::Index index(col / scale_, row / scale_);
    grid_map::Index index(col, row);
    if (isValid()) {
        CHECK(index(0) < voronoi_distance_.getSize()(0)
                  & index(1) < voronoi_distance_.getSize()(1))
        << "input: " << index(0) << ", " << index(1) << "; actual: "
        << voronoi_distance_.getSize()(0) << ", "
        << voronoi_distance_.getSize()(1);
//        return voronoi_distance_.at("distance", index)
//            * resolution_
//            * scale_;
        return voronoi_distance_.at("distance", index)
            * resolution_;
    } else {
        return 0.0;
    }
}

double VoronoiDiagram::getMinDistanceToEdgesNew(double x, double y) const {
//    grid_map::Position pt((y-map_center_(1)) / scale_, (x-map_center_(0)) / scale_);
    grid_map::Position pt((y-map_center_(1)), (x-map_center_(0)));
    if (isValid() && voronoi_distance_.isInside(pt)) {
        auto distance = voronoi_distance_.atPosition("distance",
                                                     pt,
                                                     grid_map::InterpolationMethods::INTER_LINEAR);
//        return distance * resolution_ * scale_;
        return distance * resolution_;
    }
}

double VoronoiDiagram::getMinDistanceToEdges(int row, int col) const {
    if (isValid()) {
        // 其实写反了，应该y代表row，x代表col,不过结果没错
        const int x_voronoi = row / scale_;
        const int y_voronoi = col / scale_;
        double min_distance = 0.0;

        // 从要求得点的位置一层层向外搜索，来避免遍历整个Mat；结果可能有小误差。
        for (int n = 0;; ++n) {
            const int x_smaller = std::max(x_voronoi - n, 0);
            const int y_smaller = std::max(y_voronoi - n, 0);
            const int x_larger = std::min(x_voronoi + n, voronoi_.rows);
            const int y_larger = std::min(y_voronoi + n, voronoi_.cols);
            for (int i = y_smaller; i <= y_larger; ++i) {
                if (voronoi_(x_smaller, i) == 255) {
                    min_distance = distance(x_voronoi, y_voronoi, x_smaller, i);
                    return min_distance * resolution_ * scale_;
                }
                if (voronoi_(x_larger, i) == 255) {
                    min_distance = distance(x_voronoi, y_voronoi, x_larger, i);
                    return min_distance * resolution_ * scale_;
                }
            }
            for (int i = x_smaller + 1; i <= x_larger - 1; ++i) {
                if (voronoi_(i, y_smaller) == 255) {
                    min_distance = distance(x_voronoi, y_voronoi, i, y_smaller);
                    return min_distance * resolution_ * scale_;
                }
                if (voronoi_(i, y_larger) == 255) {
                    min_distance = distance(x_voronoi, y_voronoi, i, y_larger);
                    return min_distance * resolution_ * scale_;
                }
            }
        }
    } else {
        LOG_IF(WARNING, !isValid())
        << "Voronoi DIagram is invalid, Read distance is illegal";
        return 0.0;
    }
}

void VoronoiDiagram::getPotentialField(const grid_map::GridMap &map,
                                       grid_map::GridMap *potential_field) {

    CHECK(map.exists("obstacle") & map.exists("distance"));

    potential_field->setGeometry(map.getLength(),
                                 map.getResolution(),
                                 map.getPosition());
    potential_field->setFrameId(map.getFrameId());
    potential_field->add("potential", map.get("obstacle"));

    for (grid_map::GridMapIterator it(*potential_field); !it.isPastEnd();
         ++it) {
        const double potential = getPotential(map, *it);
        const double value = potential;
        potential_field->at("potential", *it) = value;

    }
}

double VoronoiDiagram::distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void VoronoiDiagram::optimizeInput(cv::Mat1b *voro) {
    // todo: 指针如何执行重载操作符？
    for (int i = 0; i != voro->rows; ++i) {
        for (int j = 0; j != voro->cols; ++j) {
            if ((*voro)(i, j) == FREE) {
                if (i - 2 >= 0 && i + 2 != voro->rows && j - 2 >= 0
                    && j + 2 != voro->cols) {
                    if (((*voro)(i - 2, j - 2) == OCCUPANCY
                        || (*voro)(i - 1, j - 1) == OCCUPANCY)
                        && ((*voro)(i + 2, j + 2) == OCCUPANCY
                            || (*voro)(i + 1, j + 1) == OCCUPANCY)) {
                        (*voro)(i, j) = OCCUPANCY;
                        continue;
                    }
                    if (((*voro)(i, j - 2) == OCCUPANCY
                        || (*voro)(i, j - 1) == OCCUPANCY)
                        && ((*voro)(i, j + 2) == OCCUPANCY
                            || (*voro)(i, j + 1) == OCCUPANCY)) {
                        (*voro)(i, j) = OCCUPANCY;
                        continue;
                    }
                    if (((*voro)(i - 2, j) == OCCUPANCY
                        || (*voro)(i - 1, j) == OCCUPANCY)
                        && ((*voro)(i + 2, j) == OCCUPANCY
                            || (*voro)(i + 1, j) == OCCUPANCY)) {
                        (*voro)(i, j) = OCCUPANCY;
                        continue;
                    }
                }
            }
        }
    }
}

//void VoronoiDiagram::expandReducedVoronoi() {
//
//    // Create a mat of original size.
//    expanded_voronoi_.create(map_rows_, map_cols_);
//    expanded_voronoi_ = cv::Mat1b::zeros(map_rows_, map_cols_);
//    for (int i = 0; i != voronoi_.rows; ++i) {
//        for (int j = 0; j != voronoi_.cols; ++j) {
//            if (voronoi_(i, j) == FREE) {
//                int expanded_row = i * scale_ + 1;
//                int expanded_col = j * scale_ + 1;
//                expanded_voronoi_(expanded_row, expanded_col) = FREE;
//                // Connect the adjacent edge points so that the edges won't be discrete in the voronoi diagram of original size.
//                if (j + 1 < map_cols_ / scale_ && voronoi_(i, j + 1) == FREE) {
//                    for (int k = 1; k < scale_; ++k) {
//                        expanded_voronoi_(expanded_row, expanded_col + k) = FREE;
//                    }
//                }
//                if (i + 1 < map_rows_ / scale_ && j - 1 > 0 && voronoi_(i + 1, j - 1) == FREE) {
//                    for (int k = 1; k < scale_; ++k) {
//                        expanded_voronoi_(expanded_row - k, expanded_col - k) = FREE;
//                    }
//                }
//                if (i + 1 < map_rows_ / scale_ && voronoi_(i + 1, j) == FREE) {
//                    for (int k = 1; k < scale_; ++k) {
//                        expanded_voronoi_(expanded_row + k, expanded_col) = FREE;
//                    }
//                }
//                if (i + 1 < map_rows_ / scale_ && j + 1 < map_cols_ / scale_ && voronoi_(i + 1, j + 1) == FREE) {
//                    for (int k = 1; k < scale_; ++k) {
//                        expanded_voronoi_(expanded_row + k, expanded_col + k) = FREE;
//                    }
//                }
//            }
//        }
//    }
//}
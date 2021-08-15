//
// Created by lijiangnan on 19-4-14.
//

#ifndef VORONOI_VORONOI_DIAGRAM_H
#define VORONOI_VORONOI_DIAGRAM_H

#include <iostream>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>
#include <chrono>
#include "voronoi.h"

class VoronoiDiagram {
 public:
    enum GridType {
        FREE = 255,
        OCCUPANCY = 0,
    };
    VoronoiDiagram();
    VoronoiDiagram(const grid_map::GridMap &map,
                   int scale,
                   bool display = false);

    // make obstacles solid
    static void optimizeInput(cv::Mat1b *input);

    template<class cvMatType>
    static void optimizeInputNew(const cvMatType &input, cvMatType *output);

    template<class BasicType>
    void reduceScale(cv::Mat_<BasicType> &matrix,
                            int scale,
                            cv::Mat_<BasicType> *mat);

    template<class cvMatType>
    static void reduceScaleCV(const cvMatType &intput_matrix,
                              int scale,
                              cvMatType *output_mat);

    template<class cvMatType>
    static void expandReducedVoronoi(const cvMatType &reduced_voronoi,
                                     int scale,
                                     cvMatType *expanded_voronoi);


    static double distance(double x1, double y1, double x2, double y2);

    inline bool isValid() const {
        return is_valid_;
    }

    double getMinDistanceToEdges(int row, int col) const;

    double getMinDistanceToEdgesNew(int row, int col) const;

    double getMinDistanceToEdgesNew(double x, double y) const;

    double getPotential(const grid_map::GridMap &map,
                        const grid_map::Index &index) const;

    double getPotential(const grid_map::GridMap &map,
                        const grid_map::Position &pt) const;

    double getPotential(double dis_to_obs, const grid_map::Index &index) const;

    double getPotential(double dis_to_obs, const grid_map::Position &pt) const;

    // draw potential field
    void getPotentialField(const grid_map::GridMap &map,
                           grid_map::GridMap *potential_field);

private:
    bool is_valid_;
    cv::Mat1b voronoi_;
    grid_map::GridMap voronoi_distance_;
    grid_map::Position map_center_;
    int scale_;
    double resolution_;
    const double a_ = 20.0;
    const double distance_threshold_ = 6.5;
    size_t map_rows_;
    size_t map_cols_;
    bool empty_flag;
};

#include "voronoi-diagram-in.hpp"
#endif //VORONOI_VORONOI_DIAGRAM_H

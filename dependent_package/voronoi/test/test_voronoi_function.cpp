//
// Created by yangt on 19-4-22.
//
#include "voronoi/voronoi_diagram.hpp"
#include <grid_map_sdf/SignedDistanceField.hpp>

int main() {
    cv::Mat1b image = cv::imread(
            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
            cv::IMREAD_GRAYSCALE);
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(image, 0.2, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(image);
    in_gm.updateDistanceLayerCV();

    cv::Mat1b reduced_image;
    cv::resize(image, reduced_image, cv::Size(image.rows / 3, image.cols / 3));
    VoronoiDiagram voronoi_diagram(in_gm.maps, 1, true);
    grid_map::GridMap gmap;
    voronoi_diagram.getPotentialField(in_gm.maps, &gmap);
    cv::Mat potential = hmpl::eigen2cv(gmap.get("potential"));
    potential *= 255;
    cv::Mat potential_u;
    potential.convertTo(potential_u, CV_8UC1);
    cv::imshow("potential", potential_u);
    cv::moveWindow("potential", 900, 900);

    cv::Mat1b morph;
    cv::Mat kernal1 =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
    cv::morphologyEx(image, morph, cv::MORPH_OPEN, kernal1);

    VoronoiDiagram::optimizeInput(&image);

    cv::imshow("origin image", image);
    cv::imshow("reduced image", reduced_image);
    cv::moveWindow("reduced image", 560, 550);
    cv::imshow("morph image", morph);
    cv::imshow("optimized algorithm image", image);
    cv::waitKey();
}

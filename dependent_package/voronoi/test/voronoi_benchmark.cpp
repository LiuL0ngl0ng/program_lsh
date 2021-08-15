//
// Created by yangt on 19-4-21.
//

#include <opencv/cv.hpp>
#include <benchmark/benchmark.h>
#include "voronoi/voronoi_diagram.hpp"

static void BM_optimizeNew(benchmark::State &state) {
    cv::Mat1b image = cv::imread(
            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
            cv::IMREAD_GRAYSCALE);
    for (auto _:state) {
        cv::Mat1b morph;
        VoronoiDiagram::optimizeInputNew(image, &morph);
    }
}

BENCHMARK(BM_optimizeNew)->Unit(benchmark::kMillisecond);

static void BM_optimize(benchmark::State &state) {
    cv::Mat1b image = cv::imread(
            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
            cv::IMREAD_GRAYSCALE);
    for (auto _:state) {
        VoronoiDiagram::optimizeInput(&image);
    }
}

BENCHMARK(BM_optimize)->Unit(benchmark::kMillisecond);

//static void BM_reduceSize(benchmark::State &state) {
//    cv::Mat1b image = cv::imread(
//            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
//            cv::IMREAD_GRAYSCALE);
//    hmpl::InternalGridMap in_gm;
//    in_gm.initializeFromImage(image, 0.2, grid_map::Position::Zero());
//    in_gm.addObstacleLayerFromImage(image);
//    in_gm.updateDistanceLayerCV();
//    cv::Mat1b obs;
//    cv::eigen2cv(in_gm.maps.get("obstacle"), obs);
//    cv::Mat1b input_mat;
//    obs.convertTo(input_mat, CV_8UC1);
//    cv::Mat1b reduced_mat;
//    for (auto _:state) {
//        VoronoiDiagram::reduceScale(input_mat, 3, &reduced_mat);
//    }
//
//}
//BENCHMARK(BM_reduceSize)->Unit(benchmark::kMillisecond);

static void BM_reduceSizeNew(benchmark::State &state) {
    cv::Mat1b image = cv::imread(
            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
            cv::IMREAD_GRAYSCALE);
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(image, 0.2, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(image);
    in_gm.updateDistanceLayerCV();
    cv::Mat1b obs;
    cv::Mat1b reduced_mat;
    cv::eigen2cv(in_gm.maps.get("obstacle"), obs);
    cv::Mat1b input_mat;
    obs.convertTo(input_mat, CV_8UC1);
    for (auto _:state) {
        VoronoiDiagram::reduceScaleCV(input_mat,
                                      3,
                                      &reduced_mat);
    }

}
BENCHMARK(BM_reduceSizeNew)->Unit(benchmark::kMillisecond);

static void BM_establishVoronoi(benchmark::State &state) {
    cv::Mat1b image = cv::imread(
            "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
            cv::IMREAD_GRAYSCALE);
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(image, 0.2, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(image);
    in_gm.updateDistanceLayerCV();
    for (auto _:state) {
        VoronoiDiagram voronoi(in_gm.maps, 3, false);
    }
}
BENCHMARK(BM_establishVoronoi)->Unit(benchmark::kMillisecond);

//static void BM_expandeVoronoi(benchmark::State &state) {
//    cv::Mat1b image = cv::imread(
//        "/home/ljn/voronoi_update/cw_project/ivrc_ws/src/planning/space_explore/obstacles.png",
//        cv::IMREAD_GRAYSCALE);
//    hmpl::InternalGridMap in_gm;
//    in_gm.initializeFromImage(image, 0.2, grid_map::Position::Zero());
//    in_gm.addObstacleLayerFromImage(image);
//    in_gm.updateDistanceLayerCV();
//    cv::Mat1b obs;
//    cv::eigen2cv(in_gm.maps.get("obstacle"), obs);
//    cv::Mat1b input_mat;
//    obs.convertTo(input_mat, CV_8UC1);
//    cv::Mat1b reduced_mat;
//    VoronoiDiagram::reduceScale(input_mat, 5, &reduced_mat);
//    VoronoiThinner voronoi_generator;
//    bool ok = voronoi_generator.thin(reduced_mat, "zhang_suen_fast", true);
//    cv::Mat1b reduced_voronoi = voronoi_generator.get_skeleton();
//    cv::Mat1b expanded_voronoi;
//    expanded_voronoi.create(obs.rows, obs.cols);
//    for (auto _:state) {
//        VoronoiDiagram::expandReducedVoronoi(reduced_voronoi, 5, &expanded_voronoi);
//    }
//}
//BENCHMARK(BM_expandeVoronoi)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();


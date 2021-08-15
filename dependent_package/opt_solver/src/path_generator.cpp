#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <chrono>
#include <opt_utils/base.hpp>
#include "opt_solver/nlp_solver.hpp"

using hmpl::State;
using hmpl::ParamCoefficient;
using hmpl::ParamKnot;

State final_state;

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    final_state.x = goal->pose.position.x;
    final_state.y = goal->pose.position.y;
    final_state.z = tf::getYaw(goal->pose.orientation);
    final_state.k = 0;
    final_state.v = 0;
    std::cout << "get the goal." << std::endl;
}
int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "single_path_node");
    ros::NodeHandle nh("~");
    // Create publishers and subscribers
    // ros::Publisher publisher =
    // nh.advertise<geometry_msgs::PoseStamped>("pose_publisher", 1, true);
    ros::Publisher path_publisher =
            nh.advertise<nav_msgs::Path>("single_path", 1, true);

    ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    State start_state{};
    final_state.x = 10;
    final_state.y = 0;
    final_state.z = 0;

    double curvature_max = 0.5;
    double lower_bounds[3] = {-curvature_max, -curvature_max, 0.0};
    double uppper_bounds[3] = {curvature_max, curvature_max, 100};

    hmpl::NLPSolver nlp_solver(hmpl::SolverType::SNOPT);

    double last_initial_guess[3] = {};

    // Publisher in a loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        // Add data to grid map.
        ros::Time time = ros::Time::now();

        double distance = sqrt(pow(final_state.x - start_state.x, 2.0) +
                               pow(final_state.y - start_state.y, 2.0));
        lower_bounds[2] = distance;
        double param[3] = {0, 0, distance};

        // if (last_initial_guess[2] != 0) {
        //    param[0] = last_initial_guess[0];
        //    param[1] = last_initial_guess[1];
        //    param[2] = last_initial_guess[2];
        //}
        ParamKnot solution;
        auto start_time = std::chrono::steady_clock::now();
        bool res;

        if (distance == 0) {
            std::cout << "wait for inputs." << std::endl;
            res = false;
        } else {
            res = nlp_solver.solve(start_state, final_state, param,
                                   lower_bounds, uppper_bounds, &solution);
        }

        auto end_time = std::chrono::steady_clock::now();
        double elapsed_secondes =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                        end_time - start_time)
                        .count();
        if (res) {
            ROS_INFO_THROTTLE(1.0, "Problem solved.");
            ROS_INFO_THROTTLE(1.0, "Path generation time:  %f second.",
                              elapsed_secondes);
            last_initial_guess[0] = solution.p1;
            last_initial_guess[1] = solution.p2;
            last_initial_guess[2] = solution.sf;

        } else {
            ROS_INFO_THROTTLE(1.0, "There is no solution found.");
            last_initial_guess[0] = 0;
            last_initial_guess[1] = 0;
            last_initial_guess[2] = 0;
        }

        ParamCoefficient param_co = hmpl::knotToCoefficient(solution);
        std::vector<State> state_path =
                hmpl::sampleCubicSpiralByLength(start_state, param_co, 0.05);

        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pose;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        pose.header = path_msg.header;
        for (auto &point_itr : state_path) {
            pose.pose.position.x = point_itr.x;
            pose.pose.position.y = point_itr.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(point_itr.z);
            path_msg.poses.push_back(pose);
        }

        path_publisher.publish(path_msg);
        // Wait for next cycle.
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

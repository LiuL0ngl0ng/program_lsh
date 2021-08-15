#ifndef VISUAL_TOOLS_HPP
#define VISUAL_TOOLS_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>


namespace hmpl {
  // Forward declaration
  std_msgs::ColorRGBA GenerateColorRGBA(double r, double g, double b, double a = 1.0);
  // Default constants
  static const std::string RVIZ_MARKER_TOPIC = "hmpl_rviz_markers";
  static const std::string FRAME_NAME = "/map";
  static const int START_ID = 0;
  static const int GOAL_ID = 1;
  static const int SAMPLE_CIRCLE_ID = 2;
  static const int INTERNAL_PATHS = 3;
  static const int FINAL_PATH = 4;
  static const int SAMPLE_NUMBER = 72;

  // customized color
  static const std_msgs::ColorRGBA PINK = GenerateColorRGBA(0.9915, 0.4398, 0.6482);
  static const std_msgs::ColorRGBA SKY_BLUE = GenerateColorRGBA(0.2191, 0.4776, 0.94982);
  static const std_msgs::ColorRGBA LIGHT_GREEN = GenerateColorRGBA(0.2191, 0.4776, 0.94982);
  static const std_msgs::ColorRGBA BLACK = GenerateColorRGBA(0.0, 0.0, 0.0);
  static const std_msgs::ColorRGBA GREY = GenerateColorRGBA(0.7, 0.7, 0.7);
  static const std_msgs::ColorRGBA WHITE = GenerateColorRGBA(1.0, 1.0, 1.0);
  static const std_msgs::ColorRGBA CYAN = GenerateColorRGBA(0.0, 1.0, 1.0);
  static const std_msgs::ColorRGBA BLUE = GenerateColorRGBA(0, 0, 1);
  static const std_msgs::ColorRGBA DARK_CYAN = GenerateColorRGBA(0, 0.645, 0.945);
  static const std_msgs::ColorRGBA GREEN = GenerateColorRGBA(0, 1.0, 0);
  static const std_msgs::ColorRGBA RED = GenerateColorRGBA(1.0, 0, 0);

  class VisualTools {

    public:
      VisualTools(const ros::NodeHandle &nh,
                  const ros::NodeHandle &private_nh,
                  const std::string& base_frame = FRAME_NAME,
                  const std::string& marker_topic = RVIZ_MARKER_TOPIC);

      void initialize();

      std::string getMarkerTopicName();

      visualization_msgs::Marker createSpheres(
              double scale,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createLineStrip(
              double width,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createLineList(
              double width,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createCube(
              double scale,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createCylinder(
              double scale,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createDisk(
              double scale,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id);

      visualization_msgs::Marker createCircle(
              Eigen::Vector3d &point,
              double R,
              double scale,
              const std::string &ns,
              const std_msgs::ColorRGBA &color,
              const std::string &frame_id,
              int samples = SAMPLE_NUMBER);

      void publishDemoMarkers();
      void pushMarker(const visualization_msgs::Marker &marker);
      void clearMarkerArray();
      void display();
      void loadResetMarker();
      void clearDisplay();

   private:
      geometry_msgs::Point temp_point_{};
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;
      ros::Publisher rviz_pub_;
      std::string base_frame_;
      std::string marker_topic_;
      // rviz marker array
      visualization_msgs::MarkerArray rviz_marker_array_;
      // publish rate
      std::size_t subscriber_number_;
      std::size_t subscriber_number_now_;
      // reset marker
      visualization_msgs::Marker reset_marker_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW //
  };  // class VisualTools
} // namespace hmpl
#endif  // VISUAL_TOOLS_HPP

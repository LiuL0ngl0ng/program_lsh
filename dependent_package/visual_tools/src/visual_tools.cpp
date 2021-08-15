#ifndef VISUAL_TOOLS_CPP
#define VISUAL_TOOLS_CPP

#include <visual_tools/visual_tools.h>

namespace hmpl {

VisualTools::VisualTools(const ros::NodeHandle &nh,
                         const ros::NodeHandle &private_nh,
                         const std::string &base_frame,
                         const std::string &marker_topic)
    : nh_(nh),
      private_nh_(private_nh),
      base_frame_(base_frame),
      marker_topic_(marker_topic) {
    initialize();
}

void VisualTools::initialize() {
    loadResetMarker();
    rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10, false);
    clearDisplay();
}

void VisualTools::loadResetMarker() {
    reset_marker_.header.frame_id = base_frame_;
    reset_marker_.header.stamp = ros::Time();
    reset_marker_.ns = "deleteAllMarkers";  // helps during debugging
    reset_marker_.action = 3;
    reset_marker_.pose.orientation.w = 1;
}

std::string VisualTools::getMarkerTopicName() {
    return marker_topic_;
}
visualization_msgs::Marker VisualTools::createSpheres(
        double scale,
        const std::string &ns,
        const std_msgs::ColorRGBA &color,
        const std::string &frame_id) {
    visualization_msgs::Marker spheres;
    spheres.header.frame_id = frame_id;
    spheres.header.stamp = ros::Time::now();
    spheres.ns = ns;
    spheres.pose.orientation.x = 0;
    spheres.pose.orientation.y = 0;
    spheres.pose.orientation.z = 0;
    spheres.pose.orientation.w = 1;
    spheres.pose.position.x = 0;
    spheres.pose.position.y = 0;
    spheres.pose.position.z = 0;
    spheres.scale.x = scale;
    spheres.scale.y = scale;
    spheres.scale.z = scale;
    spheres.color = color;
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.points.clear();
    return spheres;
}

visualization_msgs::Marker VisualTools::createLineList(
        double width,
        const std::string &ns,
        const std_msgs::ColorRGBA &color,
        const std::string &frame_id) {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns;
    line_list.pose.orientation.x = 0;
    line_list.pose.orientation.y = 0;
    line_list.pose.orientation.z = 0;
    line_list.pose.orientation.w = 1;
    line_list.pose.position.x = 0;
    line_list.pose.position.y = 0;
    line_list.pose.position.z = 0;
    line_list.scale.x = width;
    line_list.scale.y = 1.0;
    line_list.scale.z = 1.0;
    line_list.color = color;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.points.clear();
    return line_list;
}

visualization_msgs::Marker VisualTools::createCircle(Eigen::Vector3d &center,
                                                     double R,
                                                     double scale,
                                                     const std::string &ns,
                                                     const std_msgs::ColorRGBA &color,
                                                     const std::string &frame_id,
                                                     int samples) {
    visualization_msgs::Marker circle = createLineStrip(scale, ns, color, frame_id);
    // radius R
    double delta = 2 * M_PI / samples;
    geometry_msgs::Point point{};
    point.z = 0.05;
    for (std::size_t i = 0; i <= samples; i++) {
        double theta = i * delta;
        point.x = center[0] + 0.95 * R * cos(theta);
        point.y = center[1] + 0.95 * R * sin(theta);
        circle.points.push_back(point);
    }
    return circle;
}

visualization_msgs::Marker VisualTools::createLineStrip(
        double width,
        const std::string &ns,
        const std_msgs::ColorRGBA &color,
        const std::string &frame_id) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = ns;
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;
    line_strip.pose.position.x = 0;
    line_strip.pose.position.y = 0;
    line_strip.pose.position.z = 0;
    line_strip.scale.x = width;
    line_strip.scale.y = 1.0;
    line_strip.scale.z = 1.0;
    line_strip.color = color;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.points.clear();
    return line_strip;
}

visualization_msgs::Marker VisualTools::createCube(
        double scale,
        const std::string &ns,
        const std_msgs::ColorRGBA &color,
        const std::string &frame_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.clear();
    return marker;
}



visualization_msgs::Marker VisualTools::createCylinder(
        double scale,
        const std::string &ns,
        const std_msgs::ColorRGBA &color,
        const std::string &frame_id) {
    visualization_msgs::Marker cylinder;
    cylinder.header.frame_id = frame_id;
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = ns;
    cylinder.pose.orientation.x = 0;
    cylinder.pose.orientation.y = 0;
    cylinder.pose.orientation.z = 0;
    cylinder.pose.orientation.w = 1;
    cylinder.pose.position.x = 0;
    cylinder.pose.position.y = 0;
    cylinder.pose.position.z = 0;
    cylinder.scale.x = scale;
    cylinder.scale.y = scale;
    cylinder.scale.z = scale;
    cylinder.color = color;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.points.clear();
    return cylinder;
}

visualization_msgs::Marker VisualTools::createDisk(double scale,
                                                   const std::string &ns,
                                                   const std_msgs::ColorRGBA &color,
                                                   const std::string &frame_id) {
    visualization_msgs::Marker disk = createCylinder(scale, ns, color, frame_id);
    disk.scale.z = 0.01;
    return disk;
}

void VisualTools::clearMarkerArray() {
    rviz_marker_array_.markers.clear();
}

void VisualTools::clearDisplay() {
    visualization_msgs::MarkerArray reset_marker_array;
    reset_marker_array.markers.push_back(reset_marker_);
    rviz_pub_.publish(reset_marker_array);
}

void VisualTools::pushMarker(const visualization_msgs::Marker &marker) {
    this->rviz_marker_array_.markers.push_back(marker);
}

void VisualTools::publishDemoMarkers() {
    visualization_msgs::Marker circle_marker = createCylinder(1.0, "circle", GREEN, FRAME_NAME);
    circle_marker.scale.z = 0.01;
    circle_marker.color.a = 0.2;
    this->rviz_marker_array_.markers.push_back(circle_marker);
    visualization_msgs::Marker disk_marker = createDisk(1.0, "disk", BLUE, FRAME_NAME);
    disk_marker.color.a = 0.2;
    disk_marker.pose.position.x = 0.5;
    disk_marker.pose.position.y = 0.5;
    this->rviz_marker_array_.markers.push_back(disk_marker);
}
void VisualTools::display() {
    rviz_pub_.publish(rviz_marker_array_);
}

// color generator
std_msgs::ColorRGBA GenerateColorRGBA(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

}

#endif  // VISUAL_TOOLS_CPP

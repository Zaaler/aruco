#ifndef ARUCO__ARUCO_HPP_
#define ARUCO__ARUCO_HPP_

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.h"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#include "aruco/msg/marker.hpp"

#include "../../src/ArucoDetector.cpp"
#include "../../src/ArucoMarker.cpp"
#include "../../src/ArucoMarkerInfo.cpp"

namespace aruco
{
class Aruco : public rclcpp::Node
{
public:
  explicit Aruco(const rclcpp::NodeOptions & options);
  ~Aruco() {}

private:
  double data_calibration[9];
  cv::Mat calibration;
  double data_distortion[5];
  cv::Mat distortion;
  std::vector<ArucoMarkerInfo> known;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub;
  rclcpp::Publisher<aruco::msg::Marker>::SharedPtr aruco_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
  rclcpp::Subscription<aruco::msg::Marker>::SharedPtr sub_marker_register;
  bool calibrated;
  bool use_opencv_coords;
  bool debug;
  float cosine_limit;
  float max_error_quad;
  int theshold_block_size;
  int theshold_block_size_min;
  int theshold_block_size_max;
  int min_area;
  bool first_marker_pub;

  void drawText(Mat frame, string text, Point point);
  void onFrame(const sensor_msgs::msg::Image::SharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void onMarkerRegister(const aruco::msg::Marker::SharedPtr msg);
  void stringToDoubleArray(string data, double * values, unsigned int count, string delimiter);
  void setLocalVariables();

  std::string calib_data;
  std::string distort_data;
  std::string topic_camera, topic_camera_info, topic_marker_register;
  std::string topic_pose;
};
} // namespace aruco

#endif  // ARUCO__ARUCO_HPP_

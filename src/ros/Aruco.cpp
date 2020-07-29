#include <aruco/Aruco.hpp>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace aruco
{
//Default Component
Aruco::Aruco(const rclcpp::NodeOptions & options)
: Node("aruco_node", options)
{
  data_calibration[0] = 570.3422241210938;
  data_calibration[1] = 0.;
  data_calibration[2] = 319.5;
  data_calibration[3] = 0.;
  data_calibration[4] = 570.3422241210938;
  data_calibration[5] = 239.5;
  data_calibration[6] = 0.;
  data_calibration[7] = 0.;
  data_calibration[8] = 1.;
  data_distortion[0] = 0.;
  data_distortion[1] = 0.;
  data_distortion[2] = 0.;
  data_distortion[3] = 0.;
  data_distortion[4] = 0.;
  data_distortion[5] = 0.;

  first_marker_pub = true;

  setLocalVariables();
  //Initial threshold block size
  theshold_block_size = (theshold_block_size_min + theshold_block_size_max) / 2;
  if (theshold_block_size % 2 == 0) {
    theshold_block_size++;
  }

  //Initialize calibration matrices
  calibration = Mat(3, 3, CV_64F, data_calibration);
  distortion = Mat(1, 5, CV_64F, data_distortion);

  //Camera instrinsic calibration parameters
  if (calib_data != "") {
    double values[9];
    stringToDoubleArray(calib_data, values, 9, "_");
    for (unsigned int i = 0; i < 9; i++) {
      calibration.at<double>(i / 3, i % 3) = values[i];
    }
    calibrated = true;
  }

  if (distort_data != "") {
    std::string data;
    double values[5];
    stringToDoubleArray(data, values, 5, "_");
    for (unsigned int i = 0; i < 5; i++) {
      distortion.at<double>(0, i) = values[i];
    }
    calibrated = true;
  }

  //Print all known markers
  if (debug) {
    for (unsigned int i = 0; i < known.size(); i++) {
      known[i].print();
    }
  }

  pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_pose,
    rclcpp::SensorDataQoS());
  debug_pub = this->create_publisher<sensor_msgs::msg::Image>(
    "/maruco/aruco_debug_image",
    rclcpp::QoS(10));
  aruco_pub = this->create_publisher<aruco::msg::Marker>(
    "/maruco/marker_register",
    rclcpp::SensorDataQoS());
  sub_image = this->create_subscription<sensor_msgs::msg::Image>(
    topic_camera, rclcpp::SensorDataQoS(),
    std::bind(&Aruco::onFrame, this, std::placeholders::_1));
  sub_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_camera_info, rclcpp::SensorDataQoS(),
    std::bind(&Aruco::onCameraInfo, this, std::placeholders::_1));
  sub_marker_register = this->create_subscription<aruco::msg::Marker>(
    topic_marker_register, rclcpp::SensorDataQoS(),
    std::bind(&Aruco::onMarkerRegister, this, std::placeholders::_1));
}


void Aruco::drawText(Mat frame, string text, Point point)
{
  putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2, CV_AA);
  putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1, CV_AA);
}

/**
 * Callback executed every time a new camera frame is received.
 * This callback is used to process received images and publish messages with camera position data if any.
 */
void Aruco::onFrame(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {

    if (first_marker_pub) {
      aruco::msg::Marker temp_marker;
      temp_marker.posx = 0;
      temp_marker.posy = 0;
      temp_marker.posz = 0;
      temp_marker.rotx = 0;
      temp_marker.roty = 0;
      temp_marker.rotz = 0;
      temp_marker.size = .038125;
      temp_marker.id = 401;
      aruco_pub->publish(temp_marker);
      first_marker_pub = false;
    }

    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    //Process image and get markers
    vector<ArucoMarker> markers = ArucoDetector::getMarkers(
      frame, cosine_limit,
      theshold_block_size, min_area, max_error_quad);


    //Visible
    vector<ArucoMarker> found;

    //Vector of points
    vector<Point2f> projected;
    vector<Point3f> world;

    if (markers.size() == 0) {
      theshold_block_size += 2;

      if (theshold_block_size > theshold_block_size_max) {
        theshold_block_size = theshold_block_size_min;
      }
    }

    //Check known markers and build known of points
    for (unsigned int i = 0; i < markers.size(); i++) {
      //std::cerr << "got marker: " << markers[i].id << " markers " << std::endl;
      for (unsigned int j = 0; j < known.size(); j++) {
        if (markers[i].id == known[j].id) {
          markers[i].attachInfo(known[j]);

          for (unsigned int k = 0; k < 4; k++) {
            projected.push_back(markers[i].projected[k]);
            world.push_back(known[j].world[k]);
          }

          found.push_back(markers[i]);
        }
      }
    }

    //Draw markers
    if (debug) {
      ArucoDetector::drawMarkers(frame, markers, calibration, distortion);
    }

    //Check if any marker was found
    if (world.size() > 0) {
      //Calculate position and rotation
      Mat rotation, position;

                        #if CV_MAJOR_VERSION == 2
      solvePnP(world, projected, calibration, distortion, rotation, position, false, ITERATIVE);
                        #else
      solvePnP(
        world, projected, calibration, distortion, rotation, position, false,
        SOLVEPNP_ITERATIVE);
                        #endif

      //Invert position and rotation to get camera coords
      Mat rodrigues;
      Rodrigues(rotation, rodrigues);

      Mat camera_rotation;
      Rodrigues(rodrigues.t(), camera_rotation);

      Mat camera_position = -rodrigues.t() * position;

      //Publish position and rotation
      geometry_msgs::msg::Point message_position, message_rotation;

      //Opencv coordinates
      if (use_opencv_coords) {
        message_position.x = camera_position.at<double>(0, 0);
        message_position.y = camera_position.at<double>(1, 0);
        message_position.z = camera_position.at<double>(2, 0);

        message_rotation.x = camera_rotation.at<double>(0, 0);
        message_rotation.y = camera_rotation.at<double>(1, 0);
        message_rotation.z = camera_rotation.at<double>(2, 0);
      }
      //Robot coordinates
      else {
        message_position.x = camera_position.at<double>(2, 0);
        message_position.y = -camera_position.at<double>(0, 0);
        message_position.z = -camera_position.at<double>(1, 0);

        message_rotation.x = camera_rotation.at<double>(2, 0);
        message_rotation.y = -camera_rotation.at<double>(0, 0);
        message_rotation.z = -camera_rotation.at<double>(1, 0);
      }

      // pub_position->publish(message_position);
      // pub_rotation->publish(message_rotation);

      //Publish pose
      geometry_msgs::msg::PoseStamped message_pose;

      //Header
      message_pose.header.frame_id = "aruco_observed_smm";
      //message_pose.header.seq = pub_pose_seq++;
      message_pose.header.stamp = now();

      //Position
      message_pose.pose.position.x = message_position.x;
      message_pose.pose.position.y = message_position.y;
      message_pose.pose.position.z = message_position.z;

      //Convert to quaternion
      double x = message_rotation.x;
      double y = message_rotation.y;
      double z = message_rotation.z;

      //Module of angular velocity
      double angle = sqrt(x * x + y * y + z * z);

      if (angle > 0.0) {
        message_pose.pose.orientation.x = x * sin(angle / 2.0) / angle;
        message_pose.pose.orientation.y = y * sin(angle / 2.0) / angle;
        message_pose.pose.orientation.z = z * sin(angle / 2.0) / angle;
        message_pose.pose.orientation.w = cos(angle / 2.0);
      }
      //To avoid illegal expressions
      else {
        message_pose.pose.orientation.x = 0.0;
        message_pose.pose.orientation.y = 0.0;
        message_pose.pose.orientation.z = 0.0;
        message_pose.pose.orientation.w = 1.0;
      }

      pub_pose->publish(message_pose);

      //Debug
      if (debug) {
        ArucoDetector::drawOrigin(frame, found, calibration, distortion, 0.3);

        drawText(
          frame, "Position: " + to_string(message_position.x) + ", " + to_string(
            message_position.y) + ", " + to_string(message_position.z), Point2f(10, 180));
        drawText(
          frame, "Rotation: " + to_string(message_rotation.x) + ", " + to_string(
            message_rotation.y) + ", " + to_string(message_rotation.z), Point2f(10, 200));
      }
    } else if (debug) {
      drawText(frame, "Position: unknown", Point2f(10, 180));
      drawText(frame, "Rotation: unknown", Point2f(10, 200));
    }

    //Publish visible
    std_msgs::msg::Bool message_visible;
    message_visible.data = world.size() != 0;
    // pub_visible->publish(message_visible);

    //Debug info
    if (debug) {
      drawText(frame, "Aruco ROS Debug", Point2f(10, 20));
      drawText(
        frame, "OpenCV V" + to_string(CV_MAJOR_VERSION) + "." + to_string(CV_MINOR_VERSION),
        Point2f(10, 40));
      drawText(frame, "Cosine Limit (A-Q): " + to_string(cosine_limit), Point2f(10, 60));
      drawText(frame, "Threshold Block (W-S): " + to_string(theshold_block_size), Point2f(10, 80));
      drawText(frame, "Min Area (E-D): " + to_string(min_area), Point2f(10, 100));
      drawText(frame, "MaxError PolyDP (R-F): " + to_string(max_error_quad), Point2f(10, 120));
      drawText(frame, "Visible: " + to_string(message_visible.data), Point2f(10, 140));
      drawText(frame, "Calibrated: " + to_string(calibrated), Point2f(10, 160));

      /*imshow("Aruco", frame);

      char key = (char) waitKey(1);

      if(key == 'q')
      {
              cosine_limit += 0.05;
      }
      else if(key == 'a')
      {
              cosine_limit -= 0.05;
      }

      if(key == 'w')
      {
              theshold_block_size += 2;
      }
      else if(key == 's' && theshold_block_size > 3)
      {
              theshold_block_size -= 2;
      }

      if(key == 'r')
      {
              max_error_quad += 0.005;
      }
      else if(key == 'f')
      {
              max_error_quad -= 0.005;
      }

      if(key == 'e')
      {
              min_area += 50;
      }
      else if(key == 'd')
      {
              min_area -= 50;
      }*/

      // Convert cv::Mat to Image Message
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      } catch (cv_bridge::Exception & e) {
        RCLCPP_WARN(rclcpp::get_logger(""), "cv_bridge exception: %s", e.what());
      }
      cv_ptr->image = frame;
      cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
      cv_ptr->header.stamp = now();
      cv_ptr->header.frame_id = "ar3p_camera_link_optical";
      debug_pub->publish(*cv_ptr->toImageMsg());
    }
  } catch (cv_bridge::Exception & e) {
    std::cerr << "Error getting image data" << std::endl;
  }
}

/**
 * On camera info callback.
 * Used to receive camera calibration parameters.
 */
void Aruco::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!calibrated) {
    calibrated = true;

    for (unsigned int i = 0; i < 9; i++) {
      calibration.at<double>(i / 3, i % 3) = msg->k[i];
    }

    for (unsigned int i = 0; i < 5; i++) {
      distortion.at<double>(0, i) = msg->d[i];
    }

    if (debug) {
      cout << "Camera calibration param received" << endl;
      cout << "Camera: " << calibration << endl;
      cout << "Distortion: " << distortion << endl;
    }
  }
}

/**
 * Callback to register markers on the marker list.
 * This callback received a custom marker message.
 */
void Aruco::onMarkerRegister(const aruco::msg::Marker::SharedPtr msg)
{
  for (unsigned int i = 0; i < known.size(); i++) {
    if (known[i].id == msg->id) {
      known.erase(known.begin() + i);
      //cout << "Marker " << to_string(msg->id) << " already exists, was replaced." << endl;
      break;
    }
  }

  known.push_back(
    ArucoMarkerInfo(
      msg->id, msg->size, Point3d(msg->posx, msg->posy, msg->posz),
      Point3d(msg->rotx, msg->roty, msg->rotz)));
  //cout << "Marker " << to_string(msg->id) << " added." << endl;
}

/**
 * Converts a string with numeric values separated by a delimiter to an array of double values.
 * If 0_1_2_3 and delimiter is _ array will contain {0, 1, 2, 3}.
 * @param data String to be converted
 * @param values Array to store values on
 * @param cout Number of elements in the string
 * @param delimiter Separator element
 * @return Array with values.
 */
void Aruco::stringToDoubleArray(string data, double * values, unsigned int count, string delimiter)
{
  unsigned int pos = 0, k = 0;

  while ((pos = data.find(delimiter)) != 4294967295 && k < count) {
    string token = data.substr(0, pos);
    values[k] = stod(token);
    data.erase(0, pos + delimiter.length());
    k++;
  }
}

void Aruco::setLocalVariables()
{
  //Parameters
  this->get_parameter_or<bool>("debug", debug, false);
  this->get_parameter_or<bool>("use_opencv_coords", use_opencv_coords, true);
  this->get_parameter_or<float>("cosine_limit", cosine_limit, 0.7);
  this->get_parameter_or<int>("theshold_block_size_min", theshold_block_size_min, 3);
  this->get_parameter_or<int>("theshold_block_size_max", theshold_block_size_max, 21);
  this->get_parameter_or<float>("max_error_quad", max_error_quad, 0.035);
  this->get_parameter_or<int>("min_area", min_area, 100);
  this->get_parameter_or<bool>("calibrated", calibrated, false);
  this->get_parameter_or<std::string>("calibration", calib_data, "");
  this->get_parameter_or<std::string>("distortion", distort_data, "");
  this->get_parameter_or<std::string>("topic_camera", topic_camera, "/camera1/image_raw");
  this->get_parameter_or<std::string>(
    "topic_camera_info", topic_camera_info,
    "/camera1/camera_info");
  this->get_parameter_or<std::string>(
    "topic_marker_register", topic_marker_register,
    "/maruco/marker_register");
  this->get_parameter_or<std::string>("topic_pose", topic_pose, "/maruco/aruco_pose_solution");

}

} // namespace aruco
RCLCPP_COMPONENTS_REGISTER_NODE(aruco::Aruco)

/**
 * ROS 2 USB Camera Driver node auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 7, 2023
 */

/**
 * Copyright © 2023 Intelligent Systems Lab
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <cfloat>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <usb_camera_driver/usb_camera_driver.hpp>





namespace USBCameraDriver
{

/**
 * @brief Initializes node parameters.
 */
void CameraDriverNode::init_parameters()
{
  // Register parameter updates callback
  on_set_params_chandle_ = this->add_on_set_parameters_callback(
    std::bind(
      &CameraDriverNode::on_set_parameters_callback,
      this,
      std::placeholders::_1));

  // autostart
  declare_bool_parameter(
    "autostart",
    false,
    "Start the camera driver on node initialization.",
    "cannot be changed.",
    true,
    autostart_descriptor_);

  // Base topic name
  declare_string_parameter(
    "base_topic_name",
    "camera",
    "image_transport base topic name.",
    "Cannot be changed.",
    true,
    base_topic_name_descriptor_);

  // Best-effort QoS flag
  declare_bool_parameter(
    "best_effort_qos",
    false,
    "Best-effort QoS flag.",
    "Cannot be changed.",
    true,
    base_topic_name_descriptor_);


  // Camera calibration file URL
  declare_string_parameter(
    "camera_calibration_file",
    "file://config/camera.yaml",
    "Camera calibration file URL.",
    "Cannot be changed.",
    true,
    camera_calibration_file_descriptor_);

  // Camera device ID
  declare_int_parameter(
    "camera_id",
    0, 0, INT64_MAX, 1,
    "Camera device ID.",
    "Cannot be changed.",
    true,
    camera_id_descriptor_);

  // Camera name
  declare_string_parameter(
    "camera_name",
    "camera",
    "Camera name from the configuration file.",
    "Cannot be changed.",
    true,
    camera_name_descriptor_);

  // Camera FPS
  declare_int_parameter(
    "fps",
    10, 1, 60, 1,
    "Camera FPS.",
    "Cannot be changed.",
    true,
    fps_descriptor_);

  // Image height
  declare_int_parameter(
    "image_height",
    480, 1, INT64_MAX, 1,
    "Image height.",
    "Cannot be changed.",
    true,
    image_height_descriptor_);

  // Image width
  declare_int_parameter(
    "image_width",
    640, 1, INT64_MAX, 1,
    "Image width.",
    "Cannot be changed.",
    true,
    image_width_descriptor_);

  //depht
  declare_int_parameter(
    "publisher_depth", 
    5 , 1, 100 , 1,
    "Depth of the image publisher queue.",
    "Cannot be changed.",
    true, 
    publisher_depth_descriptor_);
}

void CameraDriverNode::declare_bool_parameter(
  std::string && name,
  bool default_val,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_BOOL);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}

void CameraDriverNode::declare_double_parameter(
  std::string && name,
  double default_val, double from, double to, double step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_DOUBLE);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

void CameraDriverNode::declare_int_parameter(
  std::string && name,
  int64_t default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_INTEGER);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__integer_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

void CameraDriverNode::declare_string_parameter(
  std::string && name,
  std::string && default_val,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_STRING);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}
/**
 * @brief Parameters update validation callback.
 *
 * @param params Vector of parameters for which a change has been requested.
 * @return Operation result in SetParametersResult message.
 */
SetParametersResult CameraDriverNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // First, check if each update is feasible
  // Initial checks must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Base topic name
    if (p.get_name() == "base_topic_name") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for base_topic_name");
        break;
      }
      continue;
    }

    // Best-effort QoS flag
    if (p.get_name() == "best_effort_qos") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for best_effort_qos");
        break;
      }
      continue;
    }


    // Camera calibration file URL
    if (p.get_name() == "camera_calibration_file") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_calibration_file");
        break;
      }
      continue;
    }

    // Camera device ID
    if (p.get_name() == "camera_id") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_id");
        break;
      }
      continue;
    }

    // Camera name
    if (p.get_name() == "camera_name") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_name");
        break;
      }
      continue;
    }

    if (p.get_name() == "fps") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for fps");
        break;
      }
      continue;
    }


    // Image height
    if (p.get_name() == "image_height") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for image_height");
        break;
      }
      continue;
    }

    // Image width
    if (p.get_name() == "image_width") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for image_width");
        break;
      }
      continue;
    }

  }
  // Then, do what is necessary to update each parameter
  // Add ad-hoc update procedures must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Base topic name
    if (p.get_name() == "base_topic_name") {
      RCLCPP_INFO(
        this->get_logger(),
        "base_topic_name: %s",
        p.as_string().c_str());
      continue;
    }

    // Best-effort QoS flag
    if (p.get_name() == "best_effort_qos") {
      RCLCPP_INFO(
        this->get_logger(),
        "best_effort_qos: %s",
        p.as_bool() ? "true" : "false");
      continue;
    }


    // Camera calibration file URL
    if (p.get_name() == "camera_calibration_file") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_calibration_file: %s",
        p.as_string().c_str());
      continue;
    }

    // Camera device ID
    if (p.get_name() == "camera_id") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_id: %ld",
        p.as_int());
      continue;
    }

    // Camera name
    if (p.get_name() == "camera_name") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_name: %s",
        p.as_string().c_str());
      continue;
    }

    // Camera FPS
    if (p.get_name() == "fps") {
      fps_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "fps: %ld",
        fps_);
      continue;
    }

    // Image height
    if (p.get_name() == "image_height") {
      image_height_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "image_height: %ld",
        image_height_);
      continue;
    }

    // Image width
    if (p.get_name() == "image_width") {
      image_width_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "image_width: %ld",
        image_width_);
      continue;
    }


  }

  return res;
}
/**
 * @brief Opens the camera.
 *
 * @return True if the camera was opened successfully, false otherwise.
 */
bool CameraDriverNode::open_camera()
{
  // Open capture device
  bool opened = false;
  std::string camera_device_file = this->get_parameter("camera_device_file").as_string();
  if (camera_device_file.empty()) {
    int64_t camera_id = this->get_parameter("camera_id").as_int();
    RCLCPP_INFO(
      this->get_logger(),
      "Opening camera with ID: %ld",
      camera_id);
    opened = video_cap_.open(camera_id, cv::CAP_V4L2);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Opening camera from device file: %s",
      camera_device_file.c_str());
    opened = video_cap_.open(camera_device_file, cv::CAP_V4L2);
  }
  if (!opened ||
    !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
    !video_cap_.set(cv::CAP_PROP_FPS, fps_))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "CameraDriverNode::open_camera: Failed to open capture device");
    return false;
  }

  return true;
}

/**
 * @brief Closes the camera.
 */
void CameraDriverNode::close_camera()
{
  if (video_cap_.isOpened()) {
    video_cap_.release();
  }
}

/**
 * @brief Performs the image processing steps, between frame acquisition and messages composition.
 *
 * @return True if the frame was processed successfully, false otherwise.
 */
bool CameraDriverNode::process_frame()
{
  /**
   * The following code supports the following APIs, and self-compiles according to the
   * availability of the APIs on the system it is compiled on:
   * - OpenCV (CPU)
   * - OpenCV (CUDA)
   * - Nvidia VPI
   *
   * Independently of the APIs being used, the code is structured in the following way:
   * - Get the frame to process.
   * - Resize it to the desired format (usually not done by the USB camera, but we try to request it).
   * - If the camera has been calibrated, rectify the frame.
   * - If a rotation has been requested, rotate the frame and the rectified frame.
   * - Write final frames.
   */


  cv::resize(frame_left, frame_left, cv::Size(image_width_, image_height_));
  cv::resize(frame_right, frame_right, cv::Size(image_width_, image_height_));

  if (cinfo_manager_left->isCalibrated()) {
    cv::remap(
      frame_left,
      rectified_frame_left,
      map1_,
      map2_,
      cv::InterpolationFlags::INTER_LINEAR,
      cv::BorderTypes::BORDER_CONSTANT);
  }
  if (cinfo_manager_right->isCalibrated()) {
    cv::remap(
      frame_right,
      rectified_frame_right,
      map1_,
      map2_,
      cv::InterpolationFlags::INTER_LINEAR,
      cv::BorderTypes::BORDER_CONSTANT);
  }

  if (rotation_ == 90) {
    cv::rotate(frame_left, frame_left_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(frame_right, frame_right_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    if (cinfo_manager_left->isCalibrated()) {
      cv::rotate(rectified_frame_left, frame_left_rect_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
    if (cinfo_manager_right->isCalibrated()) {
      cv::rotate(rectified_frame_right, frame_right_rect_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
  } else if (rotation_ == -90) {
    cv::rotate(frame_left, frame_left_rot_, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(frame_right, frame_right_rot_, cv::ROTATE_90_CLOCKWISE);
    if (cinfo_manager_left->isCalibrated()) {
      cv::rotate(rectified_frame_left, frame_left_rect_rot_, cv::ROTATE_90_CLOCKWISE);
    }
     if (cinfo_manager_right->isCalibrated()) {
      cv::rotate(rectified_frame_right, frame_right_rect_rot_, cv::ROTATE_90_CLOCKWISE);
    }
  } else if (rotation_ == 180 || rotation_ == -180) {
    cv::rotate(frame_left, frame_left_rot_, cv::ROTATE_180);
    cv::rotate(frame_right, frame_right_rot_, cv::ROTATE_180);
    if (cinfo_manager_left->isCalibrated()) {
      cv::rotate(rectified_frame_left, frame_left_rect_rot_, cv::ROTATE_180);
    }
    if (cinfo_manager_right->isCalibrated()) {
      cv::rotate(rectified_frame_right, frame_right_rect_rot_, cv::ROTATE_180);
    }
  }
  return true;
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.  //controllare da alexx
 */
Image::SharedPtr CameraDriverNode::frame_to_msg(cv::Mat & frame)
{
  // Allocate new image message
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  std::memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

} // namespace USBCameraDriver

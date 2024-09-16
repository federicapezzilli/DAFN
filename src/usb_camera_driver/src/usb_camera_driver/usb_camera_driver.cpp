/**
 * ROS 2 USB Camera Driver node implementation.
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

#include <stdexcept>

#include <usb_camera_driver/usb_camera_driver.hpp>

using namespace std::chrono_literals;
namespace USBCameraDriver
{

/**
 * @brief Builds a new CameraDriverNode.
 *
 * @param opts ROS 2 node options.
 *
 * @throws RuntimeError
 */
CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions & opts)
: Node("usb_camera_driver", opts)
{
  // Initialize node parameters
  init_parameters();

  // Initialize synchronization primitives
  stopped_.store(true, std::memory_order_release);

  // Create and set up CameraInfoManager
  cinfo_manager_left = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  cinfo_manager_left->setCameraName(this->get_parameter("camera_name").as_string());
  if (!cinfo_manager_left->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }
  cinfo_manager_right = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  cinfo_manager_right->setCameraName(this->get_parameter("camera_name").as_string());
  if (!cinfo_manager_right->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }

  // Create image_transport publishers (this will use all available transports, see docs)
  uint depth = uint(this->get_parameter("publisher_depth").as_int()); //SERVE????
  camera_pub_left= std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/left/image_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      DUAQoS::Visualization::get_image_qos(depth) : //SERVE???
      DUAQoS::get_image_qos(depth)));
  camera_pub_right= std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/right/image_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      DUAQoS::Visualization::get_image_qos(depth): //SERVE???
      DUAQoS::get_image_qos(depth)));  
  rect_pub_left= std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/left/image_rect_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      //per la riduzione della qualità nel cas di mancanza di banda???
      DUAQoS::Visualization::get_image_qos(depth) :
      DUAQoS::get_image_qos(depth)));
  rect_pub_right= std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/" + this->get_parameter("base_topic_name").as_string() + "/right/image_rect_color",
      this->get_parameter("best_effort_qos").as_bool() ?
      //per la riduzione della qualità nel caso di mancanza di banda???
      DUAQoS::Visualization::get_image_qos(depth) :
      DUAQoS::get_image_qos(depth)));

  // Get and store current camera info and compute undistorsion and rectification maps  //lo lassciamo perhcè viene usto un parametro per enttrambe le fotocamere 
  if (cinfo_manager_left->isCalibrated()) {
    camera_info_left = cinfo_manager_left->getCameraInfo();

    cv::initUndistortRectifyMap(
      A_,
      D_,
      cv::Mat::eye(3, 3, CV_64F),
      A_,
      cv::Size(image_width_, image_height_),
      CV_16SC2,
      map1_,
      map2_);

  }
  if (rotation_ == 90 || rotation_ == -90) {
    camera_info_left.width = image_height_;
    camera_info_left.height = image_width_;
     camera_info_right.width = image_height_;
    camera_info_right.height = image_width_;
  } else {
    camera_info_left.width = image_width_;
    camera_info_left.height = image_height_;
    camera_info_right.width = image_width_;
    camera_info_right.height = image_height_;
  }


  // Initialize service servers
  hw_enable_server_ = this->create_service<SetBool>(
    "~/enable_camera",
    std::bind(
      &CameraDriverNode::hw_enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Node initialized");

  // Start sampling if requested
  if (this->get_parameter("autostart").as_bool()) {
    if (!open_camera()) {
      RCLCPP_FATAL(this->get_logger(), "Autostart failed: failed to open camera device");
      throw std::runtime_error("Autostart failed: failed to open camera device");
    }
    stopped_.store(false, std::memory_order_release);
    camera_sampling_thread_ = std::thread(
      &CameraDriverNode::camera_sampling_routine,
      this);
  }
}

/**
 * @brief Cleans up stuff upon node termination.
 */
CameraDriverNode::~CameraDriverNode()
{
  // Stop camera sampling thread
  bool expected = false;
  if (stopped_.compare_exchange_strong(
      expected,
      true,
      std::memory_order_release,
      std::memory_order_acquire))
  {
    camera_sampling_thread_.join();
    close_camera();
  }
  camera_pub_left->shutdown();
  camera_pub_right->shutdown();
  rect_pub_left->shutdown();
  rect_pub_right->shutdown();
  camera_pub_left.reset();
  camera_pub_right.reset();
  rect_pub_left.reset();
  rect_pub_right.reset();
  //stream_pub_left.reset();
  //stream_pub_right.reset();
  //rect_stream_pub_left.reset();
  //rect_stream_pub_right.reset();



}

/**
 * @brief Gets new frames from the camera and publishes them.
 */

void CameraDriverNode::camera_sampling_routine()
{
  // High-resolution sleep timer, in nanoseconds
  rclcpp::WallRate sampling_timer(std::chrono::nanoseconds(int(1.0 / double(fps_) * 1000000000.0)));

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread started");

  while (true) {
    // Check if thread cancellation has been requested
    if (stopped_.load(std::memory_order_acquire)) {
      break;
    }

    // Get a new frame from the camera
    video_cap_ >> frame_;

    // Process the new frame  SERVEEE???
    if (!frame_.empty()) {
      rclcpp::Time timestamp = this->get_clock()->now();

      if (!process_frame()) {
        goto sleep;
      }
      cv::Mat final_frame_left , final_frame_right, final_rect_frame_left, final_rect_frame_right;
      if (rotation_ != 0) {
        final_frame_left= frame_left_rot_;
        final_frame_right= frame_right_rot_;
        if (cinfo_manager_left->isCalibrated()) {
          final_rect_frame_left = frame_left_rect_rot_;
        }
        if (cinfo_manager_right->isCalibrated()) {
          final_rect_frame_right = frame_right_rect_rot_;
        }
      } else {
        final_frame_left= frame_left;
        final_frame_right= frame_right;
        if (cinfo_manager_left->isCalibrated()) {
          final_rect_frame_left = rectified_frame_left;
        }
        if (cinfo_manager_right->isCalibrated()) {
          final_rect_frame_right = rectified_frame_right;
        }
      }

      // Generate Image messages
      Image::SharedPtr image_left_msg= nullptr, rect_image_left_msg = nullptr;
      Image::SharedPtr image_right_msg= nullptr, rect_image_right_msg = nullptr;
      if (cinfo_manager_left->isCalibrated()) {
        rect_image_left_msg= frame_to_msg(final_rect_frame_left);
        rect_image_left_msg->header.set__stamp(timestamp);
        rect_image_left_msg->header.set__frame_id(frame_id_);
      }
      if (cinfo_manager_right->isCalibrated()) {
        rect_image_right_msg= frame_to_msg(final_rect_frame_right);
        rect_image_right_msg->header.set__stamp(timestamp);
        rect_image_right_msg->header.set__frame_id(frame_id_);
      }
      image_left_msg = frame_to_msg(final_frame_left);
      image_left_msg->header.set__stamp(timestamp);
      image_left_msg->header.set__frame_id(frame_id_);
      image_right_msg = frame_to_msg(final_frame_right);
      image_right_msg->header.set__stamp(timestamp);
      image_right_msg->header.set__frame_id(frame_id_);

      // Generate CameraInfo message
      CameraInfo::SharedPtr camera_info_left_msg = std::make_shared<CameraInfo>(camera_info_left);
      CameraInfo::SharedPtr camera_info_right_msg = std::make_shared<CameraInfo>(camera_info_right);
      camera_info_left_msg->header.set__stamp(timestamp);
      camera_info_left_msg->header.set__frame_id(frame_id_);
      camera_info_right_msg->header.set__stamp(timestamp);
      camera_info_right_msg->header.set__frame_id(frame_id_);

      // Publish new frame together with its CameraInfo on all available transports
      camera_pub_left->publish(image_left_msg, camera_info_left_msg);
      //stream_pub_left->publish(image_left_msg);
      camera_pub_right->publish(image_right_msg, camera_info_right_msg);
      //stream_pub_right->publish(image_right_msg);
      if (cinfo_manager_left->isCalibrated()) {
        rect_pub_left->publish(rect_image_left_msg);
        //rect_stream_pub_left->publish(rect_image_left_msg);
      }
      if (cinfo_manager_right->isCalibrated()) {
        rect_pub_right->publish(rect_image_right_msg);
        //rect_stream_pub_right->publish(rect_image_right_msg);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Empty frame");
    }

sleep:  
    sampling_timer.sleep();
  }

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread stopped");
}

/**
 * @brief Toggles the video capture device and related sampling thread.
 *
 * @param req Service request to parse.
 * @param resp Service response to populate.
 */
void CameraDriverNode::hw_enable_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = true;
    if (stopped_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Open camera
      if (!open_camera()) {  //QUESTA SI TROVA IN UCD_UTILS IN CUI SETTA ANCHE I PARAMETRI 
        stopped_.store(true, std::memory_order_release);
        resp->set__success(false);
        resp->set__message("Failed to open camera");
        return;
      }
      // MANCA LA PARTE  DEL SETTAGGIO DEI PARAMETRI 
      // Start camera sampling thread
      camera_sampling_thread_ = std::thread(
        &CameraDriverNode::camera_sampling_routine,
        this);
    }
    resp->set__success(true);
    resp->set__message("");
  } else {
    bool expected = false;
    if (stopped_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      camera_sampling_thread_.join();
      close_camera();
    }
    resp->set__success(true);
    resp->set__message("");
  }
}

} // namespace USBCameraDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriver::CameraDriverNode)

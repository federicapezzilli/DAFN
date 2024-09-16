/**
 * ROS 2 USB Camera Driver node.
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

#ifndef ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP
#define ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP

#ifndef DUAQOS_HPP
#define DUAQOS_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>
#include <rmw/types.h>


using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_srvs::srv;





namespace USBCameraDriver
{


// duaqos.hpp (Modifica per aggiungere Visualization)

class DUAQoS {
public:
    // Metodo statico in DUAQoS per ottenere il QoS
    static rmw_qos_profile_t get_image_qos(int depth) {
        rmw_qos_profile_t qos_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };
        return qos_profile;
    }

    // Classe annidata Visualization
    class Visualization {
    public:
        // Metodo statico in Visualization per ottenere il QoS
        static rmw_qos_profile_t get_image_qos(int depth) {
            rmw_qos_profile_t qos_profile = {
                RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth,
                RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                RMW_QOS_POLICY_DURABILITY_VOLATILE,
                RMW_QOS_DEADLINE_DEFAULT,
                RMW_QOS_LIFESPAN_DEFAULT,
                RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                false
            };
            return qos_profile;
        }
    };
};


#endif // DUAQOS_HPP


#endif // DUAQOS_HPP


/**
 * Drives USB, V4L-compatible cameras with OpenCV.
 */
class CameraDriverNode : public rclcpp::Node
{
public:
  explicit CameraDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~CameraDriverNode();

private:
  


  /* Video capture device and buffers. */
  cv::VideoCapture video_cap_;
  cv::Mat frame_, frame_left, frame_right,  frame_left_rot_ ,frame_right_rot_ ;
  cv::Mat rectified_frame_left , frame_left_rect_rot_,rectified_frame_right,  frame_right_rect_rot_;
  cv::Mat left_raw;

  /* Image processing pipeline buffers and maps*/

  cv::Mat A_, D_;
  cv::Mat map1_, map2_;

  /* Node parameters. */
  std::string frame_id_;
  int64_t fps_ = 0;
  int64_t image_height_ = 0;
  int64_t image_width_ = 0;
  int64_t rotation_ = 0;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr hw_enable_server_;

  /* Service callbacks. */
  void hw_enable_callback(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr resp);

  /*Node parameters descriptors */
  ParameterDescriptor autostart_descriptor_;
  ParameterDescriptor base_topic_name_descriptor_;
  ParameterDescriptor camera_calibration_file_descriptor_;
  ParameterDescriptor camera_id_descriptor_;
  ParameterDescriptor fps_descriptor_;
  ParameterDescriptor camera_name_descriptor_;
  ParameterDescriptor image_height_descriptor_;
  ParameterDescriptor image_width_descriptor_;
  ParameterDescriptor publisher_depth_descriptor_;
  


  /* image_transport publishers and buffers. */
  std::shared_ptr<image_transport::CameraPublisher> camera_pub_left;
  std::shared_ptr<image_transport::CameraPublisher> camera_pub_right;
  std::shared_ptr<image_transport::Publisher> rect_pub_left;
  std::shared_ptr<image_transport::Publisher> rect_pub_right;
  camera_info_manager::CameraInfo camera_info_left{};
  camera_info_manager::CameraInfo camera_info_right{};
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_left;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_right;

  /* Utility routines. */
  bool open_camera();
  void close_camera();
  bool process_frame();
  Image::SharedPtr frame_to_msg(cv::Mat & frame);

 
  void declare_bool_parameter(
    std::string && name,
    bool default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_double_parameter(
    std::string && name,
    double default_val, double from, double to, double step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_int_parameter(
    std::string && name,
    int64_t default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_string_parameter(
    std::string && name,
    std::string && default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void init_parameters();

    /* Parameters callback */
  OnSetParametersCallbackHandle::SharedPtr on_set_params_chandle_;
  SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> & params);
  /* Camera sampling thread. */
  std::thread camera_sampling_thread_;
  void camera_sampling_routine();

  /* Synchronization primitives. */
  std::atomic<bool> stopped_;
};

} // namespace USBCameraDriver


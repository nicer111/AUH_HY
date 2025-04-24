// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

    UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
            : rclcpp_lifecycle::LifecycleNode("usb_cam",
                                              rclcpp::NodeOptions(node_options).use_intra_process_comms(false)),
              m_image_msg(new sensor_msgs::msg::Image()),
              m_compressed_img_msg(nullptr),
              m_compressed_image_publisher(nullptr),
              m_compressed_cam_info_publisher(nullptr),
              m_parameters(),
              m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
              m_service_capture(
                      this->create_service<std_srvs::srv::SetBool>(
                              "set_capture",
                              std::bind(
                                      &UsbCamNode::service_capture,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3)))
    {
        m_camera  = std::make_unique<usb_cam::UsbCam>();
        // declare params
        this->declare_parameter("camera_name", "default_cam");
        this->declare_parameter("camera_info_url", "");
        this->declare_parameter("framerate", 30);
        this->declare_parameter("frame_id", "default_cam");
        this->declare_parameter("image_height", 480);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("io_method", "mmap");
        this->declare_parameter("pixel_format", "yuyv");
        this->declare_parameter("av_device_format", "YUV422P");
        this->declare_parameter("video_device", "/dev/video0");
        this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
        this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
        this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
        this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
        this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
        this->declare_parameter("auto_white_balance", true);
        this->declare_parameter("white_balance", 4000);
        this->declare_parameter("autoexposure", true);
        this->declare_parameter("exposure", 100);
        this->declare_parameter("autofocus", false);
        this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone
        m_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int64_t>(33)),
                std::bind(&UsbCamNode::update, this));
    }

    UsbCamNode::~UsbCamNode()
    {
        RCLCPP_WARN(this->get_logger(), "Shutting down");
        m_image_msg.reset();
        //m_compressed_img_msg.reset();
        m_camera_info_msg.reset();
        m_camera_info.reset();
        m_timer.reset();
        m_service_capture.reset();
        m_parameters_callback_handle.reset();

        //delete (m_camera);
    }

    void UsbCamNode::service_capture(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        (void) request_header;
        if (request->data) {
            m_camera->start_capturing();
            response->message = "Start Capturing";
        } else {
            m_camera->stop_capturing();
            response->message = "Stop Capturing";
        }
    }

    std::string resolve_device_path(const std::string & path)
    {
        if (std::filesystem::is_symlink(path)) {
            // For some reason read_symlink only returns videox
            return "/dev/" + std::string(std::filesystem::read_symlink(path));
        }
        return path;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCamNode::on_configure(
            const rclcpp_lifecycle::State&) {
        get_params();
        init();
        m_parameters_callback_handle = add_on_set_parameters_callback(
                std::bind(
                        &UsbCamNode::parameters_callback, this,
                        std::placeholders::_1));
        m_timer->cancel();
        RCLCPP_INFO(get_logger(), "usbcam on_configure() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCamNode::on_activate(
            const rclcpp_lifecycle::State&) {
        //m_camera_publisher->on_activate();
        m_camera_info_publisher->on_activate();
        //m_compressed_image_publisher->on_activate();
        //m_compressed_cam_info_publisher->on_activate();
        m_timer->reset();
        RCUTILS_LOG_INFO_NAMED(get_name(), "usb_Cam on_activate() is called.");
        //std::this_thread::sleep_for(2s);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCamNode::on_deactivate(
            const rclcpp_lifecycle::State&) {
        //m_image_publisher->on_deactivate();
        m_camera_info_publisher->on_deactivate();
        //m_compressed_image_publisher->on_deactivate();
        //m_compressed_cam_info_publisher->on_deactivate();
        m_timer->cancel();
        RCUTILS_LOG_INFO_NAMED(get_name(), "usb_Cam on_deactivate() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCamNode::on_cleanup(
            const rclcpp_lifecycle::State&) {
        if (m_camera) {
            m_camera->stop_capturing();
            m_camera->shutdown();
            m_camera.reset(); // 释放资源
        }
        if (m_timer) {
            m_timer->cancel();
            //m_timer.reset();
        }
        if (m_camera_publisher) {
            m_camera_publisher.reset();
        }

        if (m_compressed_image_publisher) {
            //m_compressed_image_publisher.reset();
        }

        if (m_compressed_cam_info_publisher) {
            //m_compressed_cam_info_publisher.reset();
        }

        if (m_camera_info_publisher) {
            m_camera_info_publisher.reset();
        }

        if (m_image_msg) {
            m_image_msg.reset();
        }

        if (m_compressed_img_msg) {
            m_compressed_img_msg.reset();
        }
        m_parameters_callback_handle.reset();

        //m_parameters_callback_handle = nullptr;
        RCUTILS_LOG_INFO_NAMED(get_name(), "usb_cam on cleanup is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCamNode::on_shutdown(
            const rclcpp_lifecycle::State& state) {

        RCUTILS_LOG_INFO_NAMED(
                get_name(),
                "usb_cam on shutdown is called from state %s.",
                state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void UsbCamNode::init()
    {
        while (m_parameters.frame_id == "") {
            RCLCPP_WARN_ONCE(
                    this->get_logger(), "Required Parameters not set...waiting until they are set");
            get_params();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        /*t_image_publisher = std::make_shared<image_transport::CameraPublisher>(
                image_transport::create_camera_publisher(
                        this->, BASE_TOPIC_NAME,
                        rclcpp::QoS(100).get_rmw_qos_profile()));*/
        if (!m_camera_publisher) {
            auto node_for_transport = std::make_shared<rclcpp::Node>("usb_cam_transport_node");

            // 使用 image_transport 创建 CameraPublisher
            image_transport::ImageTransport it(node_for_transport);

            m_camera_publisher = std::make_shared<image_transport::CameraPublisher>(
                    it.advertiseCamera(BASE_TOPIC_NAME, 10));
        }
        //m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        //        "image_raw", rclcpp::QoS(10));
        if (!m_camera_info_publisher) {
            m_camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                    "camera_info", rclcpp::QoS(10));
        }
        // load the camera info
        m_camera_info.reset(
                new camera_info_manager::CameraInfoManager(
                        this, m_parameters.camera_name, m_parameters.camera_info_url));

        // check for default camera info
        if (!m_camera_info->isCalibrated()) {
            m_camera_info->setCameraName(m_parameters.device_name);
            m_camera_info_msg->header.frame_id = m_parameters.frame_id;
            m_camera_info_msg->width = m_parameters.image_width;
            m_camera_info_msg->height = m_parameters.image_height;
            m_camera_info->setCameraInfo(*m_camera_info_msg);
        }


        // Check if given device name is an available v4l2 device
        auto available_devices = usb_cam::utils::available_devices();

        if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
            RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Device specified is not available or is not a vaild V4L2 device: `" <<
                                                                                         m_parameters.device_name << "`"
            );
            RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
            for (const auto & device : available_devices) {
                RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
                RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
            }
            rclcpp::shutdown();
            return;
        }else
        {

        }


        // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
        // and publisher
        if (m_parameters.pixel_format_name == "mjpeg" ) {
            m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
            m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
            rclcpp::QoS qos(80);
            qos.best_effort();
            qos.durability_volatile();
            m_compressed_image_publisher =
                    this->create_publisher<sensor_msgs::msg::CompressedImage>(
                            std::string(BASE_TOPIC_NAME) + "/compressed", qos);
            m_compressed_cam_info_publisher =
                    this->create_publisher<sensor_msgs::msg::CameraInfo>(
                            std::string(BASE_TOPIC_NAME) + "/compressedcamera_info", rclcpp::QoS(80));
        }
        RCLCPP_INFO(this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
                m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
                m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
                m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
        // set the IO method
        m_image_msg->header.frame_id = m_parameters.frame_id;
        io_method_t io_method =
                usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
        if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
            RCLCPP_ERROR_ONCE(
                    this->get_logger(),
                    "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
            rclcpp::shutdown();
            return;
        }
        // configure the camera
        m_camera->configure(m_parameters, io_method);
        set_v4l2_params();

        // start the camera
        m_camera->start();
        // TODO(lucasw) should this check a little faster than expected frame rate?
        // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
        const int period_ms = 1000.0 / m_parameters.framerate;
        /*m_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
                std::bind(&UsbCamNode::update, this));*/
        RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
    }

    void UsbCamNode::get_params() {
        RCLCPP_INFO(get_logger(), "Getting parameters...");
        // 使用节点自身的参数接口获取参数
        this->get_parameter("camera_name", m_parameters.camera_name);
        this->get_parameter("camera_info_url", m_parameters.camera_info_url);
        this->get_parameter("frame_id", m_parameters.frame_id);
        this->get_parameter("framerate", m_parameters.framerate);
        this->get_parameter("image_height", m_parameters.image_height);
        this->get_parameter("image_width", m_parameters.image_width);
        this->get_parameter("io_method", m_parameters.io_method_name);
        this->get_parameter("pixel_format", m_parameters.pixel_format_name);
        this->get_parameter("av_device_format", m_parameters.av_device_format);
        this->get_parameter("video_device", m_parameters.device_name);
        this->get_parameter("brightness", m_parameters.brightness);
        this->get_parameter("contrast", m_parameters.contrast);
        this->get_parameter("saturation", m_parameters.saturation);
        this->get_parameter("sharpness", m_parameters.sharpness);
        this->get_parameter("gain", m_parameters.gain);
        this->get_parameter("auto_white_balance", m_parameters.auto_white_balance);
        this->get_parameter("white_balance", m_parameters.white_balance);
        this->get_parameter("autoexposure", m_parameters.autoexposure);
        this->get_parameter("exposure", m_parameters.exposure);
        this->get_parameter("autofocus", m_parameters.autofocus);
        this->get_parameter("focus", m_parameters.focus);

        RCLCPP_INFO(get_logger(), "Parameters retrieved successfully.");
    }

    void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (auto & parameter : parameters) {
            if (parameter.get_name() == "camera_name") {
                RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
                m_parameters.camera_name = parameter.value_to_string();
            } else if (parameter.get_name() == "camera_info_url") {
                m_parameters.camera_info_url = parameter.value_to_string();
            } else if (parameter.get_name() == "frame_id") {
                m_parameters.frame_id = parameter.value_to_string();
            } else if (parameter.get_name() == "framerate") {
                RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
                m_parameters.framerate = parameter.as_double();
            } else if (parameter.get_name() == "image_height") {
                m_parameters.image_height = parameter.as_int();
            } else if (parameter.get_name() == "image_width") {
                m_parameters.image_width = parameter.as_int();
            } else if (parameter.get_name() == "io_method") {
                m_parameters.io_method_name = parameter.value_to_string();
            } else if (parameter.get_name() == "pixel_format") {
                m_parameters.pixel_format_name = parameter.value_to_string();
            } else if (parameter.get_name() == "av_device_format") {
                m_parameters.av_device_format = parameter.value_to_string();
            } else if (parameter.get_name() == "video_device") {
                m_parameters.device_name = resolve_device_path(parameter.value_to_string());
            } else if (parameter.get_name() == "brightness") {
                m_parameters.brightness = parameter.as_int();
            } else if (parameter.get_name() == "contrast") {
                m_parameters.contrast = parameter.as_int();
            } else if (parameter.get_name() == "saturation") {
                m_parameters.saturation = parameter.as_int();
            } else if (parameter.get_name() == "sharpness") {
                m_parameters.sharpness = parameter.as_int();
            } else if (parameter.get_name() == "gain") {
                m_parameters.gain = parameter.as_int();
            } else if (parameter.get_name() == "auto_white_balance") {
                m_parameters.auto_white_balance = parameter.as_bool();
            } else if (parameter.get_name() == "white_balance") {
                m_parameters.white_balance = parameter.as_int();
            } else if (parameter.get_name() == "autoexposure") {
                m_parameters.autoexposure = parameter.as_bool();
            } else if (parameter.get_name() == "exposure") {
                m_parameters.exposure = parameter.as_int();
            } else if (parameter.get_name() == "autofocus") {
                m_parameters.autofocus = parameter.as_bool();
            } else if (parameter.get_name() == "focus") {
                m_parameters.focus = parameter.as_int();
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
            }
        }
    }

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
    void UsbCamNode::set_v4l2_params()
    {
        // set camera parameters
        if (m_parameters.brightness >= 0) {
            RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
            m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
        }

        if (m_parameters.contrast >= 0) {
            RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
            m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
        }

        if (m_parameters.saturation >= 0) {
            RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
            m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
        }

        if (m_parameters.sharpness >= 0) {
            RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
            m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
        }

        if (m_parameters.gain >= 0) {
            RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
            m_camera->set_v4l_parameter("gain", m_parameters.gain);
        }

        // check auto white balance
        if (m_parameters.auto_white_balance) {
            m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
            RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
            m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
            m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
        }

        // check auto exposure
        if (!m_parameters.autoexposure) {
            RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
            RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
            // turn down exposure control (from max of 3)
            m_camera->set_v4l_parameter("exposure_auto", 1);
            // change the exposure level
            m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
        } else {
            RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
            m_camera->set_v4l_parameter("exposure_auto", 3);
        }

        // check auto focus
        if (m_parameters.autofocus) {
            m_camera->set_auto_focus(1);
            RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
            m_camera->set_v4l_parameter("focus_auto", 1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
            m_camera->set_v4l_parameter("focus_auto", 0);
            if (m_parameters.focus >= 0) {
                RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
                m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
            }
        }
    }

    bool UsbCamNode::take_and_send_image()
    {
        // Only resize if required
        if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes()) {
            m_image_msg->width = m_camera->get_image_width();
            m_image_msg->height = m_camera->get_image_height();
            m_image_msg->encoding = m_camera->get_pixel_format()->ros();
            m_image_msg->step = m_camera->get_image_step();
            if (m_image_msg->step == 0) {
                // Some formats don't have a linesize specified by v4l2
                // Fall back to manually calculating it step = size / height
                m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
            }
            m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
        }

        // grab the image, pass image msg buffer to fill
        m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));

        auto stamp = m_camera->get_image_timestamp();
        m_image_msg->header.stamp.sec = stamp.tv_sec;
        m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

        *m_camera_info_msg = m_camera_info->getCameraInfo();
        m_camera_info_msg->header = m_image_msg->header;
        m_camera_publisher->publish(*m_image_msg, *m_camera_info_msg);
        //m_image_publisher->publish(*m_image_msg);
        m_camera_info_publisher->publish(*m_camera_info_msg);
        return true;
    }

    bool UsbCamNode::take_and_send_image_mjpeg()
    {
        // Only resize if required
        if (sizeof(m_compressed_img_msg->data) != m_camera->get_image_size_in_bytes()) {
            m_compressed_img_msg->format = "jpeg";
            m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
        }

        // grab the image, pass image msg buffer to fill
        m_camera->get_image(reinterpret_cast<char *>(&m_compressed_img_msg->data[0]));

        auto stamp = m_camera->get_image_timestamp();
        m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
        m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

        *m_camera_info_msg = m_camera_info->getCameraInfo();
        m_camera_info_msg->header = m_compressed_img_msg->header;
        m_compressed_image_publisher->publish(*m_compressed_img_msg);
        m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
        return true;
    }

    rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
            const std::vector<rclcpp::Parameter> & parameters)
    {
        RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
        m_timer->reset();
        assign_params(parameters);
        set_v4l2_params();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
    }

    void UsbCamNode::update()
    {
        if (!m_camera) {
            return;
        }
        if (m_camera->is_capturing()) {
            // If the camera exposure longer higher than the framerate period
            // then that caps the framerate.
            // auto t0 = now();
            bool isSuccessful = (m_parameters.pixel_format_name == "mjpeg") ?
                                take_and_send_image_mjpeg() :
                                take_and_send_image();
            if (!isSuccessful) {
                RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
            }
        }
    }


}  // namespace usb_cam

int main(int argc, char** argv) {

    rclcpp::init(argc, argv); // 初始化ROS 2
    rclcpp::executors::SingleThreadedExecutor exe; // 创建单线程执行器
    rclcpp::NodeOptions options;

    // 如果你想设置节点名称，可以通过 NodeOptions 的构造函数或者修改 NodeOptions 对象本身
    options.arguments({"--ros-args", "--node-name", "usbcamnode"});
    auto usb_node =
            std::make_shared<usb_cam::UsbCamNode>(options);

    exe.add_node(usb_node->get_node_base_interface()); // 将节点添加到执行器
    exe.spin(); // 开始执行循环
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
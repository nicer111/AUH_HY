#ifndef LIGHT_HPP_
#define LIGHT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>
#include <wiringPi.h>
//#include <pigpio.h>

class LightNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    // 构造函数
    explicit LightNode(const std::string& node_name, bool intra_process_comms = false);

    // 生命周期回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    void brightness_callback(const std_msgs::msg::UInt8 & msg);
    void PWM_Init();
    void PWM_Set(uint8_t value);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr brightnesssub_;
    uint8_t percent = 0;
    // and it is controlled by PWM channel 0
    //const int PWM_CHANNEL_1 = 2;
    //const int PWM_CHANNEL_2 = 3;
//PIN
    const uint8_t PIN_PWM_1 = 1;
    const uint8_t PIN_PWM_2 = 24;
// This controls the max range of the PWM signal
    const int RANGE = 33750;
    //std::thread receiver_thread_;
    //std_msgs::msg::ByteMultiArray message_;
    //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> loramessagepub_;
};

#endif // LIGHT_HPP_

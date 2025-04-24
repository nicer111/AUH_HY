#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"
#include "cusmsg/msg/powerstatus.hpp"
#include "serial.h"
#include <memory>
#include <string>
#include <thread>
using namespace serial;
using cusmsg::msg::POWERSTATUS;
class BatteryNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    // 构造函数
    explicit BatteryNode(const std::string& node_name, bool intra_process_comms = false);

    // 生命周期回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    //lora接收线程绑定函数
    void batteryReceiver();
private:
    void publish_battery(const std::vector<uint8_t>& data);


    std::shared_ptr<rclcpp::TimerBase> timer_;    //定时器
    std::string usartport_;                       //串口端口
    std::int32_t usartbote_;                      //串口波特率
    std::string topic_;                           //串口消息发布话题名
    std::string gettopic_;                        //串口消息订阅话题名
    Serial * batteryserial_;
    std::thread receiver_thread_;                 //串口接收线程
    std::atomic<bool> pauseflag_ = false;         // 控制线程暂停
    std::atomic<bool> stopflag_ = false;          // 控制线程退出

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<POWERSTATUS>> powerpub_;
};

#endif // BATTERY_HPP_

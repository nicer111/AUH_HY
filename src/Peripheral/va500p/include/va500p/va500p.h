/********************************************************************************
* @author: Hu Xuanshuo
* @email: huxuanshuo2022@163.com
* @date: 25-3-28 13:00
* @version: 1.0
* @description:
********************************************************************************/
#ifndef VA500P_HPP_
#define VA500P_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "serial.h"
#include <memory>
#include <string>
#include <thread>
#include <deque>
#include <vector>
#include <algorithm>
#include <mutex>
using namespace serial;

class Va500pNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    // 构造函数
    explicit Va500pNode(const std::string& node_name, bool intra_process_comms = false);

    // 生命周期回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    //lora接收线程绑定函数
    void va500pReceiver();
private:
    void publish_depth_height(std::string & msg);
    float apply_height_filter(float raw_height, float signal_strength);

    std::shared_ptr<rclcpp::TimerBase> timer_;    //定时器
    std::string usartport_;                       //串口端口
    std::int32_t usartbote_;                      //串口波特率
    std::string topic_;                           //串口消息发布话题名
    std::string gettopic_;                        //串口消息订阅话题名
    Serial * va500pserial_;
    std::thread receiver_thread_;                 //串口接收线程
    std::atomic<bool> pauseflag_ = false;         // 控制线程暂停
    std::atomic<bool> stopflag_ = false;          // 控制线程退出

    //定深话题发布，话题名在config中的yaml里的topic
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> depthpub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> heightpub_;

    float depthcali = 0.0;          //深度标定数值，重置0深度
    float depth_ = 0.0;             //真实深度
    float height_ = 0.0;            //真实高度
    float oldheight_ = 0.0;         //上一次高度值

    struct FilterConfig {
        size_t base_window = 5;      // 基础窗口大小
        float max_rate = 1.5f;       // 最大物理变化率(m/s)
        float sample_interval = 0.1f;
        float jump_threshold = 2.0f; // 突变检测阈值(m)
    } filter_config_;

    // 滤波状态
    std::deque<float> height_window_;
    std::deque<bool> valid_flags_;  // 有效数据标记
    std::mutex filter_mutex_;

    // 信号质量评估方法
    float assess_signal_quality(const std::vector<float>& window);
};

#endif // VA500P_HPP_

#ifndef MOVECONTROLER_HPP_
#define MOVECONTROLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "cusmsg/msg/ghfpd.hpp"
#include "std_msgs/msg/float64.hpp"

using  GHFPD = cusmsg::msg::GHFPD;
using std::placeholders::_1;
class MovecontrolerNode : public rclcpp::Node {
public:
    MovecontrolerNode();

private:
    void controlLoop_V();
    void pid_Init();

    GHFPD::SharedPtr navi_;
    double depth_;
    int64_t frqv_ = 100;
    int64_t frql_ = 100;
    int64_t frqh_ = 100;
    rclcpp::Time last_time_V;
    rclcpp::Time last_time_L;
    rclcpp::Time last_time_H;

    float yawtar_ = 0.0;
    float speedtar_;
    float depthtar_;
    float heighttar_;

    rclcpp::TimerBase::SharedPtr timerv_;
    rclcpp::TimerBase::SharedPtr timercan_;
    rclcpp::TimerBase::SharedPtr timerl_;
    rclcpp::TimerBase::SharedPtr timergo_;
    rclcpp::TimerBase::SharedPtr timerdepth_;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;      //回调组
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2_;      //回调组
    rclcpp::CallbackGroup::SharedPtr callback_group_sub3_;

    rclcpp::Subscription<GHFPD>::SharedPtr ghfpdsub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depthsub_;

    std::shared_ptr<control_toolbox::PidROS> depth_pid_;
    std::shared_ptr<control_toolbox::PidROS> pitch_pid_;
    std::shared_ptr<control_toolbox::PidROS> roll_pid_;
    std::shared_ptr<control_toolbox::PidROS> speed_pid_;
    std::shared_ptr<control_toolbox::PidROS> yaw_pid_;

};



#endif // MOVECONTROLER_HPP_
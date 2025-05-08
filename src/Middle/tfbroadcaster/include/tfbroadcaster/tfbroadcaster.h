#ifndef TFBROADCASTER_HPP_
#define TFBROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "cusmsg/msg/coordinate.hpp"
#include "cusmsg/msg/ghfpd.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using COORDINATE = cusmsg::msg::COORDINATE;
using GHFPD = cusmsg::msg::GHFPD;
class TfbroadcasterNode : public rclcpp::Node {
public:
    TfbroadcasterNode();

private:
    void publish_transform();                                                //发布tf变的换函数
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    double current_roll_ = 0.0, current_pitch_ = 0.0, current_yaw_ = 0.0;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;          //tf变换广播器
    rclcpp::Subscription<COORDINATE>::SharedPtr xysub_;                      //xy订阅器坐标
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depthsub_;       //深度订阅器
    rclcpp::Subscription<GHFPD>::SharedPtr navisub_;                         //导航相关数据订阅器
    rclcpp::TimerBase::SharedPtr timer_;                                     //定时器
};



#endif // TFBROADCASTER_HPP_
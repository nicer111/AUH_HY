#ifndef CONTROLMODE_HPP_
#define CONTROLMODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "cusmsg/msg/mykey.hpp"

using std::placeholders::_1;
using MYKEY = cusmsg::msg::MYKEY;
class ControlmodeNode : public rclcpp::Node {
public:
    ControlmodeNode();

private:

    void handle_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr handlesub_;
    rclcpp::Publisher<MYKEY>::SharedPtr mykeypub_;
};



#endif // CONTROLMODE_HPP_
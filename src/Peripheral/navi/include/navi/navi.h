#ifndef NAVI_HPP_
#define NAVI_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "serial.h"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <cmath>

using namespace serial;

class NaviNode : public rclcpp::Node {
public:
    NaviNode();
private:
    void usartReceiver();                       //接收线程函数

    double yaw_;
    double pitch_;
    double roll_;
    std::string usartport_;
    std::int32_t usartbote_;
    std::thread receiver_thread_;
    Serial *my_serial_;
    int a = 0;

    const double WGS84_WIE = 7.2921151467e-5; // Earth rotation rate (rad/s)
    const double WGS84_F = 0.0033528106647474805; // Flattening
    const double WGS84_RA = 6378137.0000000000; // Semi-major axis (meters)
    const double WGS84_RB = 6356752.3142451793; // Semi-minor axis (meters)
    const double WGS84_GM0 = 398600441800000.00; // Gravitational constant
    const double WGS84_E1 = 0.0066943799901413156; // First eccentricity squared


};

#endif //NAVI_HPP_
#include <cstdio>
#include "navi/navi.h"

NaviNode::NaviNode() : Node("navinode")
{
    this->declare_parameter<std::string>("usart", "/dev/ttyAMA0");
    this->declare_parameter<std::int32_t>("bote", (115200));
    this->get_parameter("usart", usartport_);
    this->get_parameter("bote", usartbote_);

    my_serial_ = new Serial(usartport_,((uint32_t)usartbote_),Timeout::simpleTimeout(1000));
    if (my_serial_->isOpen()) {
        RCLCPP_INFO(this->get_logger(), "%s open is success", usartport_.c_str());
        receiver_thread_ = std::thread(&NaviNode::usartReceiver, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s open is error", usartport_.c_str());
    }
}

void NaviNode::usartReceiver()
{
    size_t size_;
    std::string tempbuf = "";
    std::string  dealbuf1 = "";
    while(rclcpp::ok())
    {
        size_ = my_serial_->readline(tempbuf,2048,"\r\n");
        dealbuf1 = tempbuf.substr(0,size_);
        if(size_ > 0)
        {

            //RCLCPP_INFO(this->get_logger(), "size_ :%ld %s", size_,sendbuf1.c_str());
            message_Deal(dealbuf1);
            tempbuf.clear();
            dealbuf1.clear();
            //RCLCPP_INFO(this->get_logger(), "length :%ld", size_);
        }
    }
}

void NaviNode::message_Deal(std::string & msg)
{
    if(msg.empty())
        return;
    //std::string result(msg.data.begin(), msg.data.end()-2);
    //std::string xor_ = result.substr(1, result.length() - 4);
    //uint8_t xorcheck = xorChecksum(xor_);
    std::string result = msg.substr(0,msg.size()-2);
    std::string xor_ = result.substr(1, result.length() - 4);
    std::string lastTwo = msg.substr(msg.size() - 2, 2);
    if (!std::isdigit(lastTwo[0]) || !std::isdigit(lastTwo[1])) {
        throw std::invalid_argument("navi 最后两个字符不是数字");
    }
    int check = std::stoi(lastTwo);
    if (check < 0 || check > 255) {
        throw std::out_of_range("navi 数值超出uint8_t范围");
    }
    if(!xorChecksum(xor_,static_cast<uint8_t>(check)))
    {
        RCLCPP_ERROR(this->get_logger(), "navi check error");
    }
    std::istringstream in(result);
    std::string t;
    while (std::getline(in, t, ',')) {
        v.push_back(t);
    }
    /*for (const auto& elem : v) {
        RCLCPP_INFO(this->get_logger(), "Element: '%s'", elem.c_str());
    }*/

    if (v.size() > 0) {
        if (v.at(0) != "$GHFPD") {
            RCLCPP_INFO(this->get_logger(), "navi Invalid message identifier");
            return;
        } else {
            try {
                // Ensure the vector has the expected number of elements
                if (v.size() < 16) {
                    RCLCPP_ERROR(this->get_logger(), "navi Incomplete data received");
                    return;
                }
                navidata_.type = v.at(0);
                navidata_.time = std::stod(v.at(2));
                navidata_.yaw = std::stod(v.at(3));
                navidata_.pitch = std::stod(v.at(4));
                navidata_.roll = std::stod(v.at(5));
                navidata_.latitude = std::stod(v.at(6));
                navidata_.longitude = std::stod(v.at(7));
                navidata_.altitude = std::stof(v.at(8));
                navidata_.vn = std::stof(v.at(9));
                navidata_.ve = std::stof(v.at(10));
                navidata_.vd = std::stof(v.at(11));
                navidata_.dvlbaseline = std::stof(v.at(12));
                navidata_.status = v.at(15).substr(0, 2);
                //std::string temp = v.at(15).substr(3, 5);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing values: %s", e.what());
            }
            v.clear();
        }
    }
}

void NaviNode::Eulerto_orientation(GHFPD & data)
{
    /*speed_ = std::sqrt(vn_temp * vn_temp + ve_temp * ve_temp + vd_temp * vd_temp);
    //imu_msg.linear_acceleration.x = latitude_temp;
    //imu_msg.linear_acceleration.y = longitude_temp;
    imu_msg.linear_acceleration.z = speed_;
    //imu_msg.linear_acceleration.x = std::stod(status_);  //这个取消了换了一个单独的话题
    status_msg.data = std::stod(status_);
    publisher3_->publish(status_msg);
    publisher1_->publish(imu_msg);
    Eigen::Vector3d origin_blh(29.99837285, 122.16037559, 18.353); // Latitude, Longitude, Height
    //RCLCPP_INFO(this->get_logger(), "latitude: %.10f longitude: %.10f", latitude_temp,longitude_temp);
    Eigen::Vector3d target_blh(latitude_temp, longitude_temp, 18.353);
    Eigen::Vector3d ned = blh2ned(origin_blh, target_blh);

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {ned[0],ned[1],ned[2],vn_temp,ve_temp};
    publisher2_->publish(message);

    Eigen::Matrix3d C_n_b; // NED到载体坐标系的转换矩阵
    C_n_b <<
          std::cos(pitch) * std::cos(yaw),  std::cos(pitch) * std::sin(yaw), -std::sin(pitch),
            std::sin(roll) * std::sin(pitch) * std::cos(yaw) - std::cos(roll) * std::sin(yaw),  std::sin(roll) * std::sin(pitch) * std::sin(yaw) + std::cos(roll) * std::cos(yaw), std::sin(roll) * std::cos(pitch),
            std::cos(roll) * std::sin(pitch) * std::cos(yaw) + std::sin(roll) * std::sin(yaw),  std::cos(roll) * std::sin(pitch) * std::sin(yaw) - std::sin(roll) * std::cos(yaw), std::cos(roll) * std::cos(pitch);

    Eigen::Vector3d velocity_ned(vn_temp, ve_temp, vd_temp);
    Eigen::Vector3d velocity_body = C_n_b * velocity_ned;  // 将速度从NED转换到载体坐标系

    // 发布DVL的TwistStamped消息
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.linear.x = velocity_body.x();
    twist_msg.twist.linear.y = velocity_body.y();
    twist_msg.twist.linear.z = velocity_body.z();
    dvl_twist_publisher_->publish(twist_msg);*/
}

bool NaviNode::xorChecksum(const std::string& data,uint8_t check)
{
    uint8_t checksum = 0;
    for (char byte : data) {
        checksum ^= static_cast<uint8_t>(byte);
    }
    if(check == checksum)
        return true;
    else
        return false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NaviNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


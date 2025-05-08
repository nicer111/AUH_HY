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
    ghfpdpub_ = this->create_publisher<GHFPD>("GHFPD", 10);
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
                ghfpdpub_->publish(navidata_);
                //std::string temp = v.at(15).substr(3, 5);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing values: %s", e.what());
            }
            v.clear();
        }
    }
}

void NaviNode::Coordinate_conversion(GHFPD & data)
{
    //status_msg.data = std::stod(status_);
    //publisher3_->publish(status_msg);
    //publisher1_->publish(imu_msg);
    Eigen::Vector3d origin_blh(29.99837285, 122.16037559, 18.353); // Latitude, Longitude, Height
    //RCLCPP_INFO(this->get_logger(), "latitude: %.10f longitude: %.10f", latitude_temp,longitude_temp);
    Eigen::Vector3d target_blh(data.altitude, data.longitude, data.altitude);
    Eigen::Vector3d ned = blh2ned(origin_blh, target_blh);

}

Eigen::Vector3d NaviNode::blh2ned(Eigen::Vector3d& origin_blh, Eigen::Vector3d& target_blh)
{
    double lat0 = origin_blh[0] * M_PI / 180.0;
    double lon0 = origin_blh[1] * M_PI / 180.0;
    double h0 = origin_blh[2];

    double lat = target_blh[0] * M_PI / 180.0;
    double lon = target_blh[1] * M_PI / 180.0;
    double h = target_blh[2];

    double tmp = std::sin(lat0) * std::sin(lat0);
    tmp = 1.0 - WGS84_E1 * tmp;
    double sqrttmp = std::sqrt(tmp);
    double rm = WGS84_RA * (1.0 - WGS84_E1) / (sqrttmp * tmp);
    double rn = WGS84_RA / sqrttmp;

    Eigen::Matrix<double, 3, 3> dr;
    dr.setZero();
    dr(0, 0) = rn + h0;
    dr(1, 1) = (rm + h0) * std::cos(lat0);
    dr(2, 2) = -1.0;

    Eigen::Vector3d dpos = target_blh - origin_blh;
    Eigen::Vector3d dpos_rad;
    dpos_rad << dpos[0] * M_PI / 180.0, dpos[1] * M_PI / 180.0, dpos[2];

    Eigen::Vector3d ned = dr * dpos_rad;
    ned[2] = -ned[2];

    return ned;
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


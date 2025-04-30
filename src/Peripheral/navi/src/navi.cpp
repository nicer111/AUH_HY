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
    while(rclcpp::ok())
    {
        size_ = my_serial_->readline(tempbuf,256,"\r\n");
        std::string  dealbuf1 = tempbuf.substr(0,size_);
        if(size_ > 0)
        {
            //RCLCPP_INFO(this->get_logger(), "size_ :%ld %s", size_,sendbuf1.c_str());
            tempbuf.clear();
            //RCLCPP_INFO(this->get_logger(), "length :%ld", size_);
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NaviNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
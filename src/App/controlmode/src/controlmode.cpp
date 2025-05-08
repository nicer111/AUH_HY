#include <cstdio>
#include "controlmode/controlmode.h"
ControlmodeNode::ControlmodeNode()
        : Node("controlmodenode")
{
    handlesub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
                                       std::bind(&ControlmodeNode::handle_callback, this, _1));
}

void ControlmodeNode::handle_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(msg->header.frame_id == "keyborad")
    {
        if(msg->axes[0] == 1)
            key.key_a = 1;
        else if(msg->axes[0] == -1)
            key.key_d = 1;
        if(msg->axes[1] == 1)
            key.key_w = 1;
        else if(msg->axes[1] == -1)
            key.key_s = 1;
        key.key_k = msg->buttons.at(0);
        key.key_l = msg->buttons.at(1);
        key.key_j = msg->buttons.at(3);
        key.key_i = msg->buttons.at(4);
    }
    //RCLCPP_INFO(this->get_logger(), "%s ",msg->header.frame_id.c_str());
    //RCLCPP_INFO(this->get_logger(), "%f %f",msg->axes[0],msg->axes[1]);
    //RCLCPP_INFO(this->get_logger(), "\r\n");
    //RCLCPP_INFO(this->get_logger(), "%d \r\n%d \r\n%d \r\n%d",msg->buttons.at(0),msg->buttons.at(1),msg->buttons.at(3),msg->buttons.at(4));

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlmodeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
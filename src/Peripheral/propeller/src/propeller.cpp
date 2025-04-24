#include <cstdio>
#include "propeller/propeller.h"

PropellerNode::PropellerNode()
        : Node("propellernode")
{
    getParams();
    setupCanInterface(Cannum_);
    publisherpstatus_ = this->create_publisher<PSTATUS>("PropellerStatus", 10);
    receiver_thread_ = std::thread(&PropellerNode::canReceiver, this);
    timerstatus_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 控制读取推进器状态的周期时间
            std::bind(&PropellerNode::statusLoop, this)
    );
}

void PropellerNode::getParams()
{
    if (!this->has_parameter("left_p"))
        this->declare_parameter<std::uint16_t>("left_p", 0x34E);
    else
        this->get_parameter("left_p", Left_P_);

    if (!this->has_parameter("right_p"))
        this->declare_parameter<std::uint16_t>("right_p", 0x34D);
    else
        this->get_parameter("right_p", Right_P_);

    if (!this->has_parameter("front_l_p"))
        this->declare_parameter<std::uint16_t>("front_l_p", 0x356);
    else
        this->get_parameter("front_l_p", Front_L_P_);

    if (!this->has_parameter("front_r_p"))
        this->declare_parameter<std::uint16_t>("front_r_p", 0x357);
    else
        this->get_parameter("front_r_p", Front_R_P_);

    if (!this->has_parameter("behind_l_p"))
        this->declare_parameter<std::uint16_t>("behind_l_p", 0x358);
    else
        this->get_parameter("behind_l_p", Behind_L_P_);

    if (!this->has_parameter("behind_r_p"))
        this->declare_parameter<std::uint16_t>("behind_r_p", 0x359);
    else
        this->get_parameter("behind_r_p", Behind_R_P_);


    if (!this->has_parameter("cannum"))
        this->declare_parameter<std::string>("cannum", "can0");
    else
        this->get_parameter("cannum", Cannum_);
}

void PropellerNode::setupCanInterface(const std::string &interface_name)
{
    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while opening socket");
        return;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "propeller CAN Interface Setup Success");
}

void PropellerNode::canReceiver()
{
    struct can_frame frame;
    int nbytes;

    while (rclcpp::ok())
    {
        nbytes = read(socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame");
            return;
        }
        else if (nbytes == sizeof(struct can_frame))
        {
            // 处理接收到的CAN帧
            //handleFrame(frame);
            auto data = CANDLC8();
            data.index = frame.can_id;
            data.dlc = static_cast<uint8_t>(frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; i++) {
                data.data[i] = frame.data[i];
                //std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
            }
            //std::cout << std::endl;
            //publisher1_->publish(data);
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PropellerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
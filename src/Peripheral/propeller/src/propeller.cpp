#include <cstdio>
#include "propeller/propeller.h"

PropellerNode::PropellerNode()
        : Node("propellernode")
{
    getParams();
    setupCanInterface(Cannum_);
    statuspub_ = this->create_publisher<PSTATUS>("PropellerStatus", 10);
    pstructsub_ = this->create_subscription<PSTRUCT>("Propellerctl", 10,
                               std::bind(&PropellerNode::ctl_callback, this, _1));
    receiver_thread_ = std::thread(&PropellerNode::canReceiver, this);
    timerstatus_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 控制读取推进器状态的周期时间
            std::bind(&PropellerNode::statusLoop, this)
    );
}

void PropellerNode::statusLoop()
{

}

void PropellerNode::getParams()
{
    if (!this->has_parameter("left_p"))
        this->declare_parameter<std::uint16_t>("left_p", 0x34E);
    else
        this->get_parameter("left_p", Propeller_address_[PROPELLERDEFINE::LEFT]);

    if (!this->has_parameter("right_p"))
        this->declare_parameter<std::uint16_t>("right_p", 0x34D);
    else
        this->get_parameter("right_p", Propeller_address_[PROPELLERDEFINE::RIGHT]);

    if (!this->has_parameter("front_l_p"))
        this->declare_parameter<std::uint16_t>("front_l_p", 0x356);
    else
        this->get_parameter("front_l_p", Propeller_address_[PROPELLERDEFINE::FL]);

    if (!this->has_parameter("front_r_p"))
        this->declare_parameter<std::uint16_t>("front_r_p", 0x357);
    else
        this->get_parameter("front_r_p", Propeller_address_[PROPELLERDEFINE::FR]);

    if (!this->has_parameter("behind_l_p"))
        this->declare_parameter<std::uint16_t>("behind_l_p", 0x358);
    else
        this->get_parameter("behind_l_p", Propeller_address_[PROPELLERDEFINE::BL]);

    if (!this->has_parameter("behind_r_p"))
        this->declare_parameter<std::uint16_t>("behind_r_p", 0x359);
    else
        this->get_parameter("behind_r_p", Propeller_address_[PROPELLERDEFINE::BR]);


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

void PropellerNode::ctl_callback(const PSTRUCT & msg)
{
    struct can_frame frame{};
    if(msg.status)
        frame_set(frame,Propeller_address_[msg.index],Torque_C_,msg.pthrust,8);
    else
        frame_set(frame,Propeller_address_[msg.index],Speed_C_,msg.pspeed,8);

    int bytes_sent = write(socket_, &frame, sizeof(frame));
    if (bytes_sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
    } else {
        //RCLCPP_INFO(this->get_logger(), "CAN frame send success");
    }
}

void PropellerNode::frame_set(struct can_frame & frame,const uint16_t index,const uint16_t command,int value,uint8_t dlc)
{
    frame.can_id = index;
    frame.can_dlc = dlc;
    if(dlc == 8)
    {
        frame.data[0] = command >> 8;
        frame.data[1] = command;
        frame.data[4] = value >> 24;
        frame.data[5] = value >> 16;
        frame.data[6] = value >> 8;
        frame.data[7] = value;
    }else if(dlc == 4)
    {
        frame.data[0] = command >> 8;
        frame.data[1] = command;
    }
}

void PropellerNode::message_deal(struct can_frame &frame)
{
    uint16_t command =0x0000;
    uint8_t i = 0;
    uint16_t index = frame.can_id + 128;
    /*if((msg.index+128) < 0x340)
    {
        return;
    }*/
    //RCLCPP_INFO(this->get_logger(), "ssss: %X",index);
    switch(index)
    {
        case Propeller_address_[msg.index]:
            i = 0;
            break;
        case V2_P:
            i = 1;
            break;
        case V3_P:
            i = 2;
            break;
        case V4_P:
            i = 3;
            break;
        case Left_P:
            i = 4;
            break;
        case Right_P:
            i = 5;
            break;
        default:
            break;
    }
    command = (((uint16_t)msg.data[0]) << 8) + msg.data[1];
    //RCLCPP_INFO(this->get_logger(), "dddd: %X %u %u",command,msg.data[0],msg.data[1]);
    switch(command)
    {
        case C_Speed:

            //p_status.speed[i] = msg.data[4] << 24 + msg.data[5] << 16 + msg.data[6] << 8 + msg.data[7];
            p_status.speed[i] = (int)((msg.data[4] << 24) | (msg.data[5] << 16) | (msg.data[6] << 8) | msg.data[7]);
            //p_status.speed[i] = (((int16_t)msg.data[6]) << 8) + msg.data[7];
            publisher1_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Speed: %u %u %u %u",msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
            //RCLCPP_INFO(this->get_logger(), "C_Speed: %d",p_status.speed[i]);
            break;
        case C_Current:
            p_status.current[i] = ((float)((msg.data[4] << 24) | (msg.data[5] << 16) | (msg.data[6] << 8) | (msg.data[7])))/10.0;
            //p_status.current[i] = ((float)(((int32_t)msg.data[6] << 8) + msg.data[7]))/10.0 ;
            publisher1_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Current: %f",p_status.current[i]);
            break;
            /*case C_Vol:
                p_status.vol[i] = ((float)(((int16_t)msg.data[6] << 8) + msg.data[7]))/10.0 ;
                //RCLCPP_INFO(this->get_logger(), "C_Vol: %f",p_status.vol[i]);
                break;*/
        case C_Mtemp:
            p_status.motortemp[i] = (int32_t)msg.data[7];
            publisher1_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Mtemp: %d",p_status.motortemp[i]);
            break;
        case C_Dtemp:
            p_status.drivertemp[i] = (int32_t)msg.data[7];
            publisher1_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Dtemp: %d",p_status.drivertemp[i]);
            break;
    }

    //RCLCPP_INFO(this->get_logger(), "command = %X %X %X %X %X %X %X %X %X",command,msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PropellerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
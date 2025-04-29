#include <cstdio>
#include "propeller/propeller.h"

PropellerNode::PropellerNode()
        : Node("propellernode")
{
    getParams();
    setupCanInterface(Cannum_);
    pstatuspub_ = this->create_publisher<PSTATUS>("PropellerStatus", 10);
    pstructsub_ = this->create_subscription<PSTRUCT>("Propellerctl", 10,
                               std::bind(&PropellerNode::ctl_callback, this, _1));
    receiver_thread_ = std::thread(&PropellerNode::canReceiver, this);
}


void PropellerNode::getParams()
{
    uint16_t temp;
    this->declare_parameter<std::uint16_t>("left_p", 0x34E);
    this->get_parameter("left_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::LEFT));

    this->declare_parameter<std::uint16_t>("right_p", 0x34D);
    this->get_parameter("right_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::RIGHT));

    this->declare_parameter<std::uint16_t>("front_l_p", 0x356);
    this->get_parameter("front_l_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::FL));

    this->declare_parameter<std::uint16_t>("front_r_p", 0x357);
    this->get_parameter("front_r_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::FR));

    this->declare_parameter<std::uint16_t>("behind_l_p", 0x358);
    this->get_parameter("behind_l_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::BL));

    this->declare_parameter<std::uint16_t>("behind_r_p", 0x359);
    this->get_parameter("behind_r_p", temp);
    Propeller_address_.insert(std::make_pair(temp, PROPELLERDEFINE::BR));


    this->declare_parameter<std::string>("cannum", "can0");
    this->get_parameter("cannum", Cannum_);

    for (const auto& pair : Propeller_address_) {
        Propeller_address_[pair.second] = pair.first;
    }
    //RCLCPP_INFO(this->get_logger(), "%x %x %x %x %x %x",Propeller_address_[0x34E],Propeller_address_[0x34D],Propeller_address_[0x356],Propeller_address_[0x357],Propeller_address_[0x358],Propeller_address_[0x359]);
    //RCLCPP_INFO(this->get_logger(), "%x %x %x %x %x %x",Propeller_address_[0],Propeller_address_[1],Propeller_address_[2],Propeller_address_[3],Propeller_address_[4],Propeller_address_[5]);
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
            message_deal(frame);
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
    PSTATUS p_status;
    uint16_t command =0x0000;
    uint8_t i = 0;
    uint16_t index = frame.can_id + 128;
    command = (((uint16_t)frame.data[0]) << 8) + frame.data[1];
    //RCLCPP_INFO(this->get_logger(), "dddd: %X %u %u",command,msg.data[0],msg.data[1]);
    switch(command)
    {
        case GetSpeed_C_:
            p_status.speed[Propeller_address_[index]] = (int)((frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7]);
            pstatuspub_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Speed: %u %u %u %u",msg.data[4],msg.data[5],msg.data[6],msg.data[7]);
            //RCLCPP_INFO(this->get_logger(), "C_Speed: %d",p_status.speed[i]);
            break;
        case GetCurrent_C_:
            p_status.current[i] = ((float)((frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7])))/10.0;
            pstatuspub_->publish(p_status);
            RCLCPP_INFO(this->get_logger(), "C_Current: %f",p_status.current[Propeller_address_[index]]);
            break;
        case GetTempM_C_:
            p_status.motortemp[i] = (int32_t)frame.data[7];
            pstatuspub_->publish(p_status);
            //RCLCPP_INFO(this->get_logger(), "C_Mtemp: %d",p_status.motortemp[i]);
            break;
        case GetTempE_C_:
            p_status.drivertemp[i] = (int32_t)frame.data[7];
            pstatuspub_->publish(p_status);
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
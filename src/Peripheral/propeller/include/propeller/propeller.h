#ifndef PROPELLER_HPP_
#define PROPELLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cusmsg/msg/can/candlc8.hpp"
#include "cusmsg/msg/propeller/pstatus.hpp"
#include "cusmsg/msg/propeller/propellerset.hpp"
#include <vector>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
using  CANDLC8 = cusmsg::msg::can::CANDLC8;
using  PSTATUS = cusmsg::msg::propeller::PSTATUS;
class PropellerNode : public rclcpp::Node {
public:
    PropellerNode();

private:
    void setupCanInterface(const std::string &interface_name);        //打开can接口
    void getParams();                                                 //获取相关参数
    void canReceiver();                                               //can接收线程函数
    void statusLoop();                                                //推进器状态循环can指令

    uint16_t  Front_L_P_    = 0x356;
    uint16_t  Front_R_P_    = 0x357;
    uint16_t  Behind_L_P_    = 0x358;
    uint16_t  Behind_R_P_    = 0x359;
    uint16_t  Left_P_  = 0x34E;
    uint16_t  Right_P_ = 0x34D;

    std::string Cannum_;
    std::thread receiver_thread_;
    int socket_{-1};

    rclcpp::TimerBase::SharedPtr timerstatus_;

    rclcpp::Publisher<PSTATUS>::SharedPtr publisherpstatus_;
};



#endif // PROPELLER_HPP_
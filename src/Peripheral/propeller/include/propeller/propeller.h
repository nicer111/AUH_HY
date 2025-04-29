#ifndef PROPELLER_HPP_
#define PROPELLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cusmsg/msg/candlc8.hpp"
#include "cusmsg/msg/pstatus.hpp"
#include "cusmsg/msg/propellerdefine.hpp"
#include "cusmsg/msg/propellerstruct.hpp"
#include <vector>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <map>

using  CANDLC8 = cusmsg::msg::CANDLC8;
using  PSTATUS = cusmsg::msg::PSTATUS;
using  PSTRUCT = cusmsg::msg::PROPELLERSTRUCT;
using  PROPELLERDEFINE = cusmsg::msg::PROPELLERDEFINE;
using std::placeholders::_1;
const uint16_t Torque_C_ = 0x5443;
const uint16_t Speed_C_ = 0x5643;
const uint16_t GetSpeed_C_ = 0x5156;
const uint16_t GetCurrent_C_ = 0x5143;
const uint16_t GetVol_C_ = 0x5150;
const uint16_t GetTempM_C_ = 0x5154;  //dianji
const uint16_t GetTempE_C_ = 0x4551;  //qudongqi
const uint16_t ModeQuest_C_ = 0x4D51;
const uint16_t ModeLow_C_ = 0x4D4C;
const uint16_t ModeMid_C_ = 0x4D4D;
const uint16_t ModeHigh_C_ = 0x4D48;
class PropellerNode : public rclcpp::Node {
public:
    PropellerNode();

private:
    void setupCanInterface(const std::string &interface_name);        //打开can接口
    void getParams();                                                 //获取相关参数
    void canReceiver();                                               //can接收线程函数

    void ctl_callback(const PSTRUCT & msg);                           //推进器设置回调函数
    //设置can帧
    void frame_set(struct can_frame & frame,const uint16_t index,const uint16_t command,int value,uint8_t dlc);
    void message_deal(struct can_frame & frame);

    std::map<uint16_t,uint16_t> Propeller_address_;

    std::string Cannum_;
    std::thread receiver_thread_;
    int socket_{-1};


    rclcpp::TimerBase::SharedPtr timerstatus_;               //定时器专门用来定时发送读取指令

    rclcpp::Publisher<PSTATUS>::SharedPtr pstatuspub_;        //推进器状态发布
    rclcpp::Subscription<PSTRUCT>::SharedPtr pstructsub_;    //推进器设置转速
};



#endif // PROPELLER_HPP_
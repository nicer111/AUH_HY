#ifndef MOVECONTROLER_HPP_
#define MOVECONTROLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "cusmsg/msg/ghfpd.hpp"
#include "cusmsg/msg/propellerdefine.hpp"
#include "cusmsg/msg/propellerstruct.hpp"
#include "cusmsg/msg/mykey.hpp"
#include "cusmsg/msg/movetarget.hpp"
#include "cusmsg/msg/action.hpp"
#include "cusmsg/msg/actionindex.hpp"
#include "cusmsg/msg/actionflag.hpp"
#include "std_msgs/msg/float64.hpp"

using  PSTRUCT = cusmsg::msg::PROPELLERSTRUCT;
using  PROPELLERDEFINE = cusmsg::msg::PROPELLERDEFINE;
using  GHFPD = cusmsg::msg::GHFPD;
using  MOVETARGET = cusmsg::msg::MOVETARGET;
using  ACTION = cusmsg::msg::ACTION;
using  ACTIONFLAG = cusmsg::msg::ACTIONFLAG;                 //用于PID区别 推进器方向
using  ACTIONINDEX = cusmsg::msg::ACTIONINDEX;
using  MYKEY = cusmsg::msg::MYKEY;
using  std::placeholders::_1;
class MovecontrolerNode : public rclcpp::Node {
public:
    MovecontrolerNode();

private:
    void controlLoop_V();                                 //垂直推进器相关的pid都在这里
    void controlLoop_L();                                 //水平推进器相关的pid都在这里
    void pid_Init();
    void setThrusterSpeeds_L(double speed_effort, double yaw_effort);
    void setThrusterSpeeds_V(double depth_effort, double roll_effort, double pitch_effort);
    void setThrust(uint16_t index,int16_t thrust);       //设置推进器的函数
    void keyMove(const MYKEY & key);
    void msgAction(const ACTION & act);
    void setTurn(double tar_yaw, double cur_yaw,uint8_t action_index);
    void setDive(uint8_t action_index);
    void setComeup(uint8_t action_index);

    MOVETARGET  target_;                                  //所有目标值的存储
    GHFPD navi_;                                          //包含很多相关导航信息，imu，dvl等等
    double depth_;                                        //读取到的值深度数据
    int64_t frqv_ = 100;                                  //垂直定时器初始化个给了值，主要还是靠配置文件的值
    int64_t frql_ = 100;                                  //水平定时器初始化个给了值，主要还是靠配置文件的值
    int64_t frqh_ = 100;                                  //初始化个给了值，主要还是靠配置文件的值
    rclcpp::Time last_time_V;                             //垂直推进器pid的使用时间
    rclcpp::Time last_time_L;                             //水平推进器pid的使用时间
    rclcpp::Time last_time_H;

    double depthstep_ = 0.1;

    double speedmax_ = 1.0;
    double speedstep_ = 0.01;

    uint8_t last_index = 0;
    uint8_t turn_flag = 0;
    uint8_t depth_flag = 0;
    int str_flag = 0;
    float yawtar_ = 0.0;
    float speedtar_;
    float depthtar_ = 5;
    float heighttar_;

    const double limit_v = 60.0;
    const double limit_l = 98.0;

    rclcpp::TimerBase::SharedPtr timerv_;                  //depth、roll、pitch的pid定时器
    rclcpp::TimerBase::SharedPtr timerl_;                  //yaw、speed的pid定时器
    rclcpp::TimerBase::SharedPtr timergo_;
    rclcpp::TimerBase::SharedPtr timerdepth_;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;               //回调组1
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2_;               //回调组2
    rclcpp::CallbackGroup::SharedPtr callback_group_sub3_;               //回调组3
    rclcpp::CallbackGroup::SharedPtr callback_group_sub4_;               //回调组3

    rclcpp::Subscription<GHFPD>::SharedPtr ghfpdsub_;                    //导航相关数订阅
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depthsub_;   //深度值订阅
    rclcpp::Subscription<MYKEY>::SharedPtr mykeysub_;                    //键盘订阅
    rclcpp::Subscription<ACTION>::SharedPtr actionsub_;                  //动作订阅

    rclcpp::Publisher<PSTRUCT>::SharedPtr pstructpub_;                   //推进器设置动起来的话题发布

    std::shared_ptr<control_toolbox::PidROS> depth_pid_;
    std::shared_ptr<control_toolbox::PidROS> pitch_pid_;
    std::shared_ptr<control_toolbox::PidROS> roll_pid_;
    std::shared_ptr<control_toolbox::PidROS> speed_pid_;
    std::shared_ptr<control_toolbox::PidROS> yaw_pid_;

};



#endif // MOVECONTROLER_HPP_
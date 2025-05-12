#include <cstdio>
#include "movecontroler/movecontroler.h"

MovecontrolerNode::MovecontrolerNode()
        : Node("movecontrolernode")
{
    callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_sub1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_sub2_;
    auto sub3_opt = rclcpp::SubscriptionOptions();
    sub3_opt.callback_group = callback_group_sub3_;
    auto sub4_opt = rclcpp::SubscriptionOptions();
    sub4_opt.callback_group = callback_group_sub4_;

    pstructpub_ = this->create_publisher<PSTRUCT>("Propellerctl", 10);

    this->declare_parameter<std::int64_t>("frqv", 100);
    this->declare_parameter<std::int64_t>("frql", 100);
    this->declare_parameter<std::int64_t>("frqh", 100);
    frqv_ = this->get_parameter("frqv").get_value<int64_t>();
    frql_ = this->get_parameter("frql").get_value<int64_t>();
    frqh_ = this->get_parameter("frqh").get_value<int64_t>();

    ghfpdsub_ = create_subscription<GHFPD>(
            "GHFPD", 10,
            [this](const GHFPD::SharedPtr msg) {
                //std::memcpy(&navi_,msg.get(),sizeof(GHFPD));
                navi_ = *msg;
            },sub1_opt);
    depthsub_ = create_subscription<std_msgs::msg::Float64>(
            "depth", 10,
            [this](const std_msgs::msg::Float64 msg) {
                depth_ = msg.data;
            },sub1_opt);
    mykeysub_ = this->create_subscription<MYKEY>("mykey", 10,
                                                 std::bind(&MovecontrolerNode::keyMove, this, _1),sub3_opt);

    actionsub_ = this->create_subscription<ACTION>("action", 10,
                                                   std::bind(&MovecontrolerNode::msgAction, this, _1),sub4_opt);

    pid_Init();
    timerv_ = this->create_wall_timer(
            std::chrono::milliseconds(frqv_),  // 控制周期，根据需要调整
            std::bind(&MovecontrolerNode::controlLoop_V, this),
            callback_group_sub2_
    );

    timerl_ = this->create_wall_timer(
            std::chrono::milliseconds(frql_),  // 控制周期，根据需要调整
            std::bind(&MovecontrolerNode::controlLoop_L, this),
            callback_group_sub3_
    );

    timerv_->reset();
    timerl_->reset();
}

void MovecontrolerNode::msgAction(const ACTION & act)
{
        switch(act.index)
        {
            case ACTIONINDEX::IDEL:
                RCLCPP_INFO(this->get_logger(), "IDEL");
                break;
            case ACTIONINDEX::TURN:
                setTurn(act.target.yawtar,navi_.yaw,act.index);
                break;
            case ACTIONINDEX::FORWARD:
                RCLCPP_INFO(this->get_logger(), "FORWARD");
                break;
            case ACTIONINDEX::BACKWARD:
                RCLCPP_INFO(this->get_logger(), "BACKWARD");
                break;
            case ACTIONINDEX::DIVE:
                setDive(act.index);
                break;
            case ACTIONINDEX::DIVE_FORWARD:
                RCLCPP_INFO(this->get_logger(), "DIVE_FORWARD");
                break;
            case ACTIONINDEX::DIVE_BACKWARD:
                RCLCPP_INFO(this->get_logger(), "DIVE_BACKWARD");
                break;
            case ACTIONINDEX::COME_UP:
                setComeup(act.index);
                break;
            case ACTIONINDEX::STOP:
                RCLCPP_INFO(this->get_logger(), "STOP");
                break;
            default:
                break;
        }
    last_index = act.index;
}

void MovecontrolerNode::keyMove(const MYKEY & key)
{
    if(!key.key_space)
    {
        //前进
        if(key.key_w && !key.key_s)
        {
            target_.yawtar = navi_.yaw;
            if(target_.speedtar < speedmax_)
            {
                target_.speedtar += speedstep_;
            }else
            {
                target_.speedtar = speedmax_;
            }
        }
        //后退
        if(key.key_s && !key.key_w)
        {
            target_.yawtar = navi_.yaw;
            if(target_.speedtar < speedmax_)
            {
                target_.speedtar += speedstep_;
            }else
            {
                target_.speedtar = speedmax_;
            }
        }
        //左转
        if(key.key_a && !key.key_d)
        {
            target_.yawtar = navi_.yaw + 1.0;
            if(target_.yawtar > 360) target_.yawtar -= 360;
        }
        //右转
        if(key.key_d && !key.key_a)
        {
            target_.yawtar = navi_.yaw - 1.0;
            if(target_.yawtar < 0) target_.yawtar += 360;
        }
        //上浮
        if(key.key_i && !key.key_k)
        {
            target_.yawtar = navi_.yaw;
            if(target_.depthtar >= depthstep_)
            {
                target_.depthtar -= depthstep_;
            }else
            {
                target_.depthtar = 0.0;
            }
        }
        //定深
        if(key.key_k && !key.key_i)
        {
            target_.yawtar = navi_.yaw;
            target_.depthtar += depthstep_;
        }
    }else
    {
        target_.speedtar = 0;
        //RCLCPP_INFO(this->get_logger(), "stop all");
    }
    //RCLCPP_INFO(this->get_logger(), "depthtar: %f yawtar: %f",target_.depthtar,target_.yawtar);
    //RCLCPP_INFO(this->get_logger(), "speedtar: %f",target_.speedtar);
}

void MovecontrolerNode::controlLoop_V()
{
    if (last_time_V.nanoseconds() == 0) {
        last_time_V = this->get_clock()->now();
        return;  // 跳过首次无效计算
    }

    auto current_time_v = this->get_clock()->now();
    rclcpp::Duration dt = current_time_v - last_time_V; // 计算时间差
    if(dt.seconds() > 1)
    {
        last_time_V = this->get_clock()->now();
        return;
    }
    double depth_effort = depth_pid_->compute_command(depthtar_ - depth_, dt);
//    if(depth_effort >= limit_v)
//    {
//        depth_effort = limit_v;
//    }else if(depth_effort < -limit_v)
//    {
//        depth_effort = -limit_v;
//    }
    //RCLCPP_INFO(this->get_logger(), "effort = %f",depth_effort);
    //RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll_, pitch_, yaw_);

    double roll_effort = roll_pid_->compute_command(0.0 - navi_.roll, dt);
//    if(roll_effort >= limit_v)
//    {
//        roll_effort = limit_v;
//    }else if(roll_effort < -limit_v)
//    {
//        roll_effort = -limit_v;
//    }
    double pitch_effort = pitch_pid_->compute_command(0.0 - navi_.pitch, dt);
//    if(pitch_effort >= limit_v)
//    {
//        pitch_effort = limit_v;
//    }else if(pitch_effort < -limit_v)
//    {
//        pitch_effort = -limit_v;
//    }
    //RCLCPP_INFO(this->get_logger(), "effort =%f  %f   %f",depth_effort,roll_effort,pitch_effort);
    setThrusterSpeeds_V(depth_effort,roll_effort,pitch_effort);
    last_time_V = current_time_v;
    //RCLCPP_INFO(this->get_logger(), "PID RUN");
}


void MovecontrolerNode::controlLoop_L()
{
    if (last_time_L.nanoseconds() == 0) {
        last_time_L = this->get_clock()->now();
        return;  // 跳过首次无效计算
    }

    auto current_time_l = this->get_clock()->now();
    rclcpp::Duration dt = current_time_l - last_time_L; // 计算时间差
    if(dt.seconds() > 1)
    {
        last_time_L = this->get_clock()->now();
        return;
    }
    double speed_effort = speed_pid_->compute_command(speedtar_  - navi_.speed, dt);
    if(speed_effort >= limit_l)
    {
        speed_effort = limit_l;
    }else if(speed_effort < -limit_l)
    {
        speed_effort = -limit_l;
    }
    if(speedtar_ == 0.0)
    {
        speed_effort = 0;
    }
    double yaw_error = yawtar_ - navi_.yaw;
    while (yaw_error > 360) yaw_error -= 360;
    while (yaw_error < 0) yaw_error += 360;
    double yaw_effort = yaw_pid_->compute_command(yaw_error, dt);

    //RCLCPP_INFO(this->get_logger(), "effort = %f %f",speed_effort,yaw_effort);
    setThrusterSpeeds_L(speed_effort,yaw_effort);
    last_time_L = current_time_l;
}

void MovecontrolerNode::setThrusterSpeeds_L(double speed_effort, double yaw_effort)
{
    int left_target;
    int right_target;
    if(str_flag == -1){
            left_target =  speed_effort + yaw_effort;//speed_effort + yaw_effort;
            right_target =  speed_effort - yaw_effort;//speed_effort + yaw_effort;
    }else if(str_flag == 1){
        left_target  =  -speed_effort + yaw_effort;//speed_effort + yaw_effort;
        right_target =  -speed_effort - yaw_effort;//speed_effort + yaw_effort;
    }
    setThrust(PROPELLERDEFINE::LEFT,left_target);
    setThrust(PROPELLERDEFINE::RIGHT,right_target);
    //RCLCPP_INFO(this->get_logger(), "vertical: %f %f",verticalLetf_target,verticalRight_target);

}

void MovecontrolerNode::setThrusterSpeeds_V(double depth_effort, double roll_effort, double pitch_effort)
{
    if(depth_flag != ACTIONFLAG ::DIVEBACKWARD_FLAG) {
        int16_t fl_target = depth_effort + roll_effort +
                            pitch_effort;//- compensationpower[compensation_index_];;//depth_effort + roll_effort;
        int16_t fr_target = depth_effort - roll_effort +
                            pitch_effort;//- compensationpower[compensation_index_];;//depth_effort - roll_effort;
        int16_t bl_target = depth_effort + roll_effort - pitch_effort;
        int16_t br_target = depth_effort - roll_effort - pitch_effort;
        setThrust(PROPELLERDEFINE::FL, fl_target);
        setThrust(PROPELLERDEFINE::FR, fr_target);
        setThrust(PROPELLERDEFINE::BL, bl_target);
        setThrust(PROPELLERDEFINE::BR, br_target);
    }
    //pstructpub_->publish(pstruct);
//    msg.data = "set_ctlspeed " + V1_P + " " + std::to_string(vertical1_target);
//    //RCLCPP_INFO(this->get_logger(), "%s",msg.data.c_str());
//    publisher_->publish(msg);
//    msg.data = "set_ctlspeed " + V2_P + " " + std::to_string(vertical2_target);
//    //RCLCPP_INFO(this->get_logger(), "%s",msg1.data.c_str());
//    publisher_->publish(msg);
//    msg.data = "set_ctlspeed " + V3_P + " " + std::to_string(vertical3_target);
//    //RCLCPP_INFO(this->get_logger(), "%s",msg1.data.c_str());
//    publisher_->publish(msg);
//    msg.data = "set_ctlspeed " + V4_P + " " + std::to_string(vertical4_target);
//    //RCLCPP_INFO(this->get_logger(), "%s",msg1.data.c_str());
//    publisher_->publish(msg);
}

void MovecontrolerNode::setThrust(uint16_t index,int16_t thrust)
{
    PSTRUCT pstruct;
    pstruct.index = index;
    pstruct.pthrust = thrust;
    pstruct.status = true;
    pstructpub_->publish(pstruct);
}

void MovecontrolerNode::pid_Init()
{
    depth_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Depth_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );
    /*height_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Height_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );*/

    roll_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Roll_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );

    pitch_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Pitch_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );
    speed_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Speed_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );
    yaw_pid_ = std::make_shared<control_toolbox::PidROS>(
            this->get_node_base_interface(),
            this->get_node_logging_interface(),
            this->get_node_parameters_interface(),
            this->get_node_topics_interface(),
            "Yaw_pid",  // 这里的prefix用于参数的命名空间，例如`speed_pid.kp`
            true          // 表示前缀仅用于参数
    );

    if(!depth_pid_->initialize_from_ros_parameters())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize depth_pid_ parameters!");
    }
    if(!roll_pid_->initialize_from_ros_parameters())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize roll_pid_ parameters!");
    }
    if(!pitch_pid_->initialize_from_ros_parameters())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize pitch_pid_ parameters!");
    }
    if(!speed_pid_->initialize_from_ros_parameters())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize speed_pid_ parameters!");
    }
    if(!yaw_pid_->initialize_from_ros_parameters())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize yaw_pid_ parameters!");
    }
    this->get_parameter("frqv",frqv_);
    this->get_parameter("frql",frql_);
    this->get_parameter("frqh",frqh_);
    RCLCPP_INFO(this->get_logger(), "%ld %ld %ld",frqv_,frql_,frqh_);

}

void MovecontrolerNode::setTurn(double tar_yaw, double cur_yaw,uint8_t action_index)
{
    double delta = fmod(tar_yaw - cur_yaw + 180.0, 360.0) - 180.0;
    double angle = fabs(delta);
    const double epsilon = 1e-6; // 浮点误差阈值
    if (delta > epsilon) {
        turn_flag = ACTIONFLAG::TURNRIGHT_FLAG;
    } else if (delta < -epsilon) {
        turn_flag = ACTIONFLAG::TURNLEFT_FLAG;;
    }else
    {
        turn_flag = ACTIONFLAG::TURNRIGHT_FLAG;
    }
    if (fabs(angle - 180.0) < epsilon) {
        turn_flag = ACTIONFLAG::TURNRIGHT_FLAG;
    }
    if(turn_flag)
    {
        RCLCPP_INFO(this->get_logger(), "turn_right  to  %lf",tar_yaw);
    }else
    {
        RCLCPP_INFO(this->get_logger(), "turn_left  to  %lf",tar_yaw);
    }
    if(    last_index == action_index
       || (last_index == ACTIONINDEX::FORWARD)
       || (last_index == ACTIONINDEX::BACKWARD)
       || (last_index == ACTIONINDEX::DIVE_FORWARD)
       || (last_index == ACTIONINDEX::DIVE_BACKWARD))
    {
        yaw_pid_->reset(true);
    }else
    {
        yaw_pid_->reset();
    }
    timerv_.reset();
}

void MovecontrolerNode::setDive(uint8_t action_index)
{
    if(    last_index == action_index
           || (last_index == ACTIONINDEX::DIVE_FORWARD)
           || (last_index == ACTIONINDEX::DIVE_BACKWARD)
           || (last_index == ACTIONINDEX::COME_UP))
    {
        yaw_pid_->reset(true);
        roll_pid_->reset(true);
        pitch_pid_->reset(true);
        timerv_.reset();
    }else
    {
        yaw_pid_->reset();
        roll_pid_->reset();
        pitch_pid_->reset();
    }
    depth_flag = ACTIONFLAG::DIVE_FLAG;
}

void MovecontrolerNode::setComeup(uint8_t action_index)
{
    if(    last_index == action_index
           || (last_index == ACTIONINDEX::DIVE_FORWARD)
           || (last_index == ACTIONINDEX::DIVE_BACKWARD)
           || (last_index == ACTIONINDEX::DIVE))
    {
        yaw_pid_->reset(true);
        roll_pid_->reset(true);
        pitch_pid_->reset(true);
        timerv_.reset();
    }else
    {
        yaw_pid_->reset();
        roll_pid_->reset();
        pitch_pid_->reset();
    }
    depth_flag = ACTIONFLAG::COMEUP_FLAG;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovecontrolerNode>();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

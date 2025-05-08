#include <cstdio>
#include "movecontroler/movecontroler.h"

MovecontrolerNode::MovecontrolerNode()
        : Node("movecontrolernode")
{
    callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_sub1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_sub2_;
    auto sub3_opt = rclcpp::SubscriptionOptions();
    sub3_opt.callback_group = callback_group_sub3_;

    this->declare_parameter<std::int64_t>("frqv", 100);
    this->declare_parameter<std::int64_t>("frql", 100);
    this->declare_parameter<std::int64_t>("frqh", 100);
    frqv_ = this->get_parameter("frqv").get_value<int64_t>();
    frql_ = this->get_parameter("frql").get_value<int64_t>();
    frqh_ = this->get_parameter("frqh").get_value<int64_t>();

    ghfpdsub_ = create_subscription<GHFPD>(
            "GHFPD", 10,
            [this](const GHFPD::SharedPtr msg) {
                navi_ = msg;
            },sub1_opt);
    depthsub_ = create_subscription<std_msgs::msg::Float64>(
            "depth", 10,
            [this](const std_msgs::msg::Float64 msg) {
                depth_ = msg.data;
            },sub1_opt);

    pid_Init();
    timerv_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 控制周期，根据需要调整
            std::bind(&MovecontrolerNode::controlLoop_V, this),
            callback_group_sub2_
    );

}

void MovecontrolerNode::controlLoop_V()
{

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
    //新版本的这个模块修改了初始化的函数
    //depth_pid_->initPid(Depth_kp, Depth_ki, Depth_kd, 0.0, 0.0,true); // 这里的最后两个参数是积分限制和微分限制，暂时设置为0
    //height_pid_->initPid(Height_kp, Height_ki, Height_kd, 0.0, 0.0,true);
    //RCLCPP_INFO(this->get_logger(), "pid = %f %f %f",Depth_kp,Depth_ki,Depth_kd);
    //roll_pid_->initPid(Roll_kp, Roll_ki, Roll_kd, 0.0, 0.0,true);
    //pitch_pid_->initPid(Pitch_kp, Pitch_ki, Pitch_kd, 0.0, 0.0,true);
    //speed_pid_->initPid(Speed_kp, Speed_ki, Speed_kd, 0.0, 0.0,true);
    //yaw_pid_->initPid(Yaw_kp, Yaw_ki, Yaw_kd, 0.0, 0.0,true);
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

#include "light/light.h"
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;

// 构造函数实现
LightNode::LightNode(const std::string& node_name, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

void LightNode::PWM_Init()
{
    if (wiringPiSetup() == -1) {
        RCLCPP_ERROR(get_logger(), "wiringpi init failed");
        return ;
    }
    pinMode(PIN_PWM_1, PWM_OUTPUT);
    pinMode(PIN_PWM_2, PWM_OUTPUT);
    pwmSetClock(16);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(RANGE);
    /*if (!bcm2835_init())
        return;
    bcm2835_gpio_fsel(PIN_PWM_1, BCM2835_GPIO_FSEL_ALT5);
    bcm2835_gpio_fsel(PIN_PWM_2, BCM2835_GPIO_FSEL_ALT5);
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);
    bcm2835_pwm_set_mode(PWM_CHANNEL_1, 1, 1);
    bcm2835_pwm_set_mode(PWM_CHANNEL_2, 1, 1);
    bcm2835_pwm_set_range(PWM_CHANNEL_1, RANGE);
    bcm2835_pwm_set_range(PWM_CHANNEL_2, RANGE);
    PWM_Set(0);*/
}

void LightNode::PWM_Set(uint8_t value)
{
    //gpioPWM(18, value * 255);
    //bcm2835_pwm_set_data(PWM_CHANNEL_1, (value * RANGE)/100); //value 11~20有效
    //bcm2835_pwm_set_data(PWM_CHANNEL_2, (value * RANGE)/100);
    pwmWrite(PIN_PWM_1, (value * RANGE)/100);
    pwmWrite(PIN_PWM_2, (value * RANGE)/100);
}

void LightNode::brightness_callback(const std_msgs::msg::UInt8 & msg)
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
        RCLCPP_WARN(get_logger(), "Received message in non-active state, ignoring");
        return;
    }
    uint8_t value = msg.data;
    RCLCPP_INFO(this->get_logger(), "亮度百分比： '%u'", msg.data);
    PWM_Set(value);
}

// 生命周期回调函数实现
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LightNode::on_configure(
        const rclcpp_lifecycle::State&) {
    brightnesssub_ = this->create_subscription<std_msgs::msg::UInt8>("topic_brightness", 10, std::bind(&LightNode::brightness_callback, this, _1));
    PWM_Init();
    //loramessagepub_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);
    /*timer_ = this->create_wall_timer(
            1s, std::bind(&LoraNode::publish, this));*/
    RCLCPP_INFO(get_logger(), "light on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LightNode::on_activate(
        const rclcpp_lifecycle::State&) {

    RCUTILS_LOG_INFO_NAMED(get_name(), "light on_activate() is called.");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LightNode::on_deactivate(
        const rclcpp_lifecycle::State&) {
    PWM_Set(0);
    RCUTILS_LOG_INFO_NAMED(get_name(), "light on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LightNode::on_cleanup(
        const rclcpp_lifecycle::State&) {
    brightnesssub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "light on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LightNode::on_shutdown(
        const rclcpp_lifecycle::State& state) {
    brightnesssub_.reset();
    RCUTILS_LOG_INFO_NAMED(
            get_name(),
            "light on shutdown is called from state %s.",
            state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv); // 初始化ROS 2
    rclcpp::executors::SingleThreadedExecutor exe; // 创建单线程执行器
    std::shared_ptr<LightNode> lc_node =
            std::make_shared<LightNode>("lightnode"); // 创建LifecycleTalker节点

    exe.add_node(lc_node->get_node_base_interface()); // 将节点添加到执行器
    exe.spin(); // 开始执行循环
    rclcpp::shutdown();
    return 0;
}

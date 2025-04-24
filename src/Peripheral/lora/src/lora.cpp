#include "lora/lora.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

// 构造函数实现
LoraNode::LoraNode(const std::string& node_name, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

// 消息发布实现
void LoraNode::publish()
{
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

    /*if (!pub_->is_activated()) {
        RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
        RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    }

    pub_->publish(std::move(msg));*/
}

void LoraNode::loraReceiver()
{
    size_t size;
    std::string tempbuf = "";
    std_msgs::msg::String message;

    while (!stopflag_) {
        if (pauseflag_) {
            std::this_thread::sleep_for(100ms); // 如果暂停，则休眠避免高 CPU 占用
            continue;
        }

        if (loraserial_ && loraserial_->isOpen()) {
            try {
                size = loraserial_->readline(tempbuf, 256, "\r\n");
                std::string sendbuf1 = tempbuf.substr(0, size);
                if (size > 0) {
                    RCLCPP_INFO(this->get_logger(), "size_ :%ld %s", size, sendbuf1.c_str());
                    message.data = sendbuf1;
                    loramessagepub_->publish(message);
                    message.data.clear();
                    tempbuf.clear();
                }
            } catch (const std::exception &e) {
                message.data.clear();
                RCLCPP_ERROR(this->get_logger(), "Exception during serial read: %s", e.what());
            }
        } else {
            std::this_thread::sleep_for(500ms); // 如果串口关闭，降低循环频率
        }
    }
}

// 生命周期回调函数实现
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LoraNode::on_configure(
        const rclcpp_lifecycle::State&) {
    if (!this->has_parameter("usart")) {
        this->declare_parameter<std::string>("usart", "/dev/ttyAMA0");
    }
    if (!this->has_parameter("topic")) {
        this->declare_parameter<std::string>("topic", "usartmessage");
    }
    if (!this->has_parameter("gettopic")) {
        this->declare_parameter<std::string>("gettopic", "usartsend");
    }
    if (!this->has_parameter("bote")) {
        this->declare_parameter<std::int32_t>("bote", 115200);
    }
    this->get_parameter("usart", usartport_);
    this->get_parameter("topic", topic_);
    this->get_parameter("gettopic", gettopic_);
    this->get_parameter("bote", usartbote_);
    stopflag_ = false;
    pauseflag_ = true;
    RCLCPP_INFO(this->get_logger(), "%s %d %s %s",usartport_.c_str(),usartbote_,topic_.c_str(),gettopic_.c_str());
    if(loraserial_ == nullptr)
    {
        loraserial_ = new Serial(usartport_,((uint32_t)usartbote_),Timeout::simpleTimeout(1000));
    }else
    {
        loraserial_->open();
    }
    if (loraserial_->isOpen()) {
        RCLCPP_INFO(this->get_logger(), "%s open is success", usartport_.c_str());
        if (!receiver_thread_.joinable()) { // 确保线程只启动一次
            receiver_thread_ = std::thread(&LoraNode::loraReceiver, this);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s open is error", usartport_.c_str());
    }
    loramessagepub_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);
    /*timer_ = this->create_wall_timer(
            1s, std::bind(&LoraNode::publish, this));*/
    RCLCPP_INFO(get_logger(), "lora on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LoraNode::on_activate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = false;
    loramessagepub_->on_activate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "lora on_activate() is called.");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LoraNode::on_deactivate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = true;  // 暂停线程
    loramessagepub_->on_deactivate();
    if (loraserial_ && loraserial_->isOpen()) {
        loraserial_->close();  // 关闭串口
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "lora on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LoraNode::on_cleanup(
        const rclcpp_lifecycle::State&) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    if (loraserial_ != nullptr) {
        delete loraserial_;
        loraserial_ = nullptr;
    }
    loramessagepub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "lora on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LoraNode::on_shutdown(
        const rclcpp_lifecycle::State& state) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    loramessagepub_.reset();
    RCUTILS_LOG_INFO_NAMED(
            get_name(),
            "lora on shutdown is called from state %s.",
            state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv); // 初始化ROS 2
    rclcpp::executors::SingleThreadedExecutor exe; // 创建单线程执行器
    std::shared_ptr<LoraNode> lc_node =
            std::make_shared<LoraNode>("loranode"); // 创建LifecycleTalker节点

    exe.add_node(lc_node->get_node_base_interface()); // 将节点添加到执行器
    exe.spin(); // 开始执行循环
    rclcpp::shutdown();

    return 0;
}
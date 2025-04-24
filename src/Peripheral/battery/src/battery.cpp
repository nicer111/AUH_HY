#include "battery/battery.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

// 构造函数实现
BatteryNode::BatteryNode(const std::string& node_name, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

void BatteryNode::publish_battery(const std::vector<uint8_t>& data)
{
    POWERSTATUS power_;
    uint16_t Vbat = 0;
    int16_t Abat = 0;
    uint16_t RSOC = 100;
    uint32_t RemCapacity = 0;
    uint8_t Chg_FET_stat = 1;
    uint8_t DisChg_FET_stat = 1;

    if (data.size() < 13) {
        RCLCPP_ERROR(this->get_logger(), "data size error");
        return;
    }

    switch (data.at(2)) {
        case 0x90:
            Vbat = (data[4] << 8) | data[5];
            Abat = (data[8] << 8) | data[9];
            Abat = 30000 - Abat;
            RSOC = (data[10] << 8) | data[11];
            power_.power = static_cast<float>(RSOC) / 10.0f;
            power_.voltage = static_cast<float>(Vbat) / 10.0f;
            power_.current = static_cast<float>(Abat) / 10.0f;
            //publisher1_->publish(power_);
            //RCLCPP_INFO(this->get_logger(), "power:%f vol:%f cur:%f", power_.power, power_.voltage, power_.current);
            break;
        case 0x93:
            RemCapacity = (data[9] << 16) | (data[10] << 8) | data[11];
            Chg_FET_stat = data[5];
            DisChg_FET_stat = data[6];
            RCLCPP_INFO(this->get_logger(), "state:%u reserve:%u", DisChg_FET_stat, RemCapacity);
            break;
    }
}

void BatteryNode::batteryReceiver()
{
    size_t size;
    std::vector<uint8_t> my_data;
    uint8_t temp;
    while (!stopflag_) {
        if (pauseflag_) {
            std::this_thread::sleep_for(100ms); // 如果暂停，则休眠避免高 CPU 占用
            continue;
        }

        if (batteryserial_ && batteryserial_->isOpen()) {
            try {
                size = batteryserial_->read(&temp, 1);
                if (size > 0)
                {
                    my_data.push_back(temp);
                    if(my_data.at(0) == 0xA5)
                    {
                        if(my_data.size() >= 2)
                        {
                            if(my_data.at(1) == 0x01)
                            {
                                if(my_data.size() >= 13)
                                {
                                    publish_battery(my_data);
                                    my_data.clear();
                                }
                            }else
                            {
                                batteryserial_->flushInput();
                                my_data.clear();
                            }
                        }
                    }else
                    {
                        batteryserial_->flushInput();
                        my_data.clear();
                    }
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during serial read: %s", e.what());
            }
        } else {
            std::this_thread::sleep_for(500ms); // 如果串口关闭，降低循环频率
        }
    }
}

// 生命周期回调函数实现
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BatteryNode::on_configure(
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
    if(batteryserial_ == nullptr)
    {
        batteryserial_ = new Serial(usartport_,((uint32_t)usartbote_),Timeout::simpleTimeout(1000));
    }else
    {
        batteryserial_->open();
    }
    if (batteryserial_->isOpen()) {
        RCLCPP_INFO(this->get_logger(), "%s open is success", usartport_.c_str());
        if (!receiver_thread_.joinable()) { // 确保线程只启动一次
            receiver_thread_ = std::thread(&BatteryNode::batteryReceiver, this);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s open is error", usartport_.c_str());
    }
    powerpub_ = this->create_publisher<POWERSTATUS>(topic_, 10);
    /*timer_ = this->create_wall_timer(
            1s, std::bind(&LoraNode::publish, this));*/
    RCLCPP_INFO(get_logger(), "battery on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BatteryNode::on_activate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = false;
    powerpub_->on_activate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "battery on_activate() is called.");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BatteryNode::on_deactivate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = true;  // 暂停线程
    powerpub_->on_deactivate();
    if (batteryserial_ && batteryserial_->isOpen()) {
        batteryserial_->close();  // 关闭串口
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "battery on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BatteryNode::on_cleanup(
        const rclcpp_lifecycle::State&) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    if (batteryserial_ != nullptr) {
        delete batteryserial_;
        batteryserial_ = nullptr;
    }
    powerpub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "battery on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BatteryNode::on_shutdown(
        const rclcpp_lifecycle::State& state) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    powerpub_.reset();
    RCUTILS_LOG_INFO_NAMED(
            get_name(),
            "battery on shutdown is called from state %s.",
            state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv); // 初始化ROS 2
    rclcpp::executors::SingleThreadedExecutor exe; // 创建单线程执行器
    std::shared_ptr<BatteryNode> lc_node =
            std::make_shared<BatteryNode>("batterynode"); // 创建LifecycleTalker节点

    exe.add_node(lc_node->get_node_base_interface()); // 将节点添加到执行器
    exe.spin(); // 开始执行循环
    rclcpp::shutdown();

    return 0;
}
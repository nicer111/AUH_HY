#include "va500p/va500p.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

// 构造函数实现
Va500pNode::Va500pNode(const std::string& node_name, bool intra_process_comms)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}


void Va500pNode::publish_depth_height(std::string & msg)
{
    std::istringstream in(msg);
    std::vector<std::string> v;
    std::string t;
    while (std::getline(in, t, ',')) {
        v.push_back(t);
    }
    try {
        if (v.size() < 4) {
            RCLCPP_ERROR(this->get_logger(), "depth data received error");
            return;
        }
        float height = atof(v.at(1).c_str());
        float Par = atof(v.at(3).c_str());
        //RCLCPP_INFO(this->get_logger(), "Par = %f\r\n",Par);
        depth_ = (Par - 10.068) * 10.0 / 9.80;
        //RCLCPP_INFO(this->get_logger(), "depth = %f ,height = %f\r\n",depth,height);
        std_msgs::msg::Float64 depth_msg;
        depth_msg.data = depth_ - depthcali;
        depthpub_->publish(depth_msg);

        std::vector<float> recent_heights(height_window_.begin(), height_window_.end());
        recent_heights.push_back(height);
        const float signal_quality = assess_signal_quality(recent_heights);

        // 应用自适应滤波
        height_ = apply_height_filter(height, signal_quality);
        std_msgs::msg::Float64 height_msg;
        height_msg.data = height_;
        heightpub_->publish(height_msg);
    } catch (const std::exception& e) {
        return ;
    }
}

float Va500pNode::assess_signal_quality(const std::vector<float>& window) {
    if(window.size() < 3) return 1.0f; // 数据不足时默认高质量

    // 计算短期波动
    float sum_diff = 0;
    for(size_t i=1; i<window.size(); ++i){
        sum_diff += fabs(window[i] - window[i-1]);
    }
    const float avg_diff = sum_diff / (window.size()-1);

    // 计算异常值密度
    std::vector<float> sorted(window);
    std::sort(sorted.begin(), sorted.end());
    const float median = sorted[sorted.size()/2];
    int outliers = 0;
    for(float h : window){
        if(fabs(h - median) > filter_config_.jump_threshold) outliers++;
    }
    const float outlier_ratio = (float)outliers / window.size();

    // 综合质量评估
    const float stability = exp(-avg_diff * 2.0f);     // 波动越小分越高
    const float consistency = 1.0f - outlier_ratio;   // 异常值越少分越高
    return (stability * 0.6f + consistency * 0.4f);
}

float Va500pNode::apply_height_filter(float raw_height, float signal_quality) {
    std::lock_guard<std::mutex> lock(filter_mutex_);

    // 动态调整窗口大小
    int adaptive_window = filter_config_.base_window +
                          static_cast<int>((1.0f - signal_quality) * 6);
    adaptive_window = std::clamp(adaptive_window, 3, 15);

    // 更新数据窗口
    height_window_.push_back(raw_height);
    while(height_window_.size() > adaptive_window) {
        height_window_.pop_front();
    }

    // 多级滤波核心
    std::vector<float> sorted(height_window_.begin(), height_window_.end());
    std::sort(sorted.begin(), sorted.end());

    // 计算动态中值 (去除10%极端值)
    const size_t trim = sorted.size() * 0.1f;
    std::vector<float> trimmed(sorted.begin()+trim, sorted.end()-trim);
    const float robust_median = trimmed[trimmed.size()/2];

    // 物理变化率约束
    static float last_valid = robust_median;
    const float max_change = filter_config_.max_rate *
                             filter_config_.sample_interval *
                             (1.0f + signal_quality); // 质量好时放宽限制
    float filtered = robust_median;
    if(fabs(filtered - last_valid) > max_change) {
        filtered = last_valid + ((filtered > last_valid) ? max_change : -max_change);
    }
    last_valid = filtered;

    return filtered;
}

void Va500pNode::va500pReceiver()
{
    size_t size;
    std::string tempbuf = "";
    std::string sendbuf1 = "";
    while (!stopflag_) {
        if (pauseflag_) {
            std::this_thread::sleep_for(100ms); // 如果暂停，则休眠避免高 CPU 占用
            continue;
        }

        if (va500pserial_ && va500pserial_->isOpen()) {
            try {
                size = va500pserial_->readline(tempbuf, 256, "\r\n");
                sendbuf1 = tempbuf.substr(0, size);
                if (size > 0) {
                    RCLCPP_INFO(this->get_logger(), "size_ :%ld %s", size, sendbuf1.c_str());
                    publish_depth_height(sendbuf1);
                    tempbuf.clear();
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
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Va500pNode::on_configure(
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
    if(va500pserial_ == nullptr)
    {
        va500pserial_ = new Serial(usartport_,((uint32_t)usartbote_),Timeout::simpleTimeout(1000));
    }else
    {
        va500pserial_->open();
    }
    if (va500pserial_->isOpen()) {
        RCLCPP_INFO(this->get_logger(), "%s open is success", usartport_.c_str());
        if (!receiver_thread_.joinable()) { // 确保线程只启动一次
            receiver_thread_ = std::thread(&Va500pNode::va500pReceiver, this);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s open is error", usartport_.c_str());
    }
    depthpub_ = this->create_publisher<std_msgs::msg::Float64>(topic_, 10);
    heightpub_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    /*timer_ = this->create_wall_timer(
            1s, std::bind(&LoraNode::publish, this));*/
    RCLCPP_INFO(get_logger(), "depth on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Va500pNode::on_activate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = false;
    depthpub_->on_activate();
    heightpub_->on_activate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "depth on_activate() is called.");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Va500pNode::on_deactivate(
        const rclcpp_lifecycle::State&) {
    pauseflag_ = true;  // 暂停线程
    depthpub_->on_deactivate();
    heightpub_->on_deactivate();
    if (va500pserial_ && va500pserial_->isOpen()) {
        va500pserial_->close();  // 关闭串口
    }
    RCUTILS_LOG_INFO_NAMED(get_name(), "depth on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Va500pNode::on_cleanup(
        const rclcpp_lifecycle::State&) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    if (va500pserial_ != nullptr) {
        delete va500pserial_;
        va500pserial_ = nullptr;
    }
    depthpub_.reset();
    heightpub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "depth on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Va500pNode::on_shutdown(
        const rclcpp_lifecycle::State& state) {
    stopflag_ = true;  // 停止线程
    if (receiver_thread_.joinable()) {
        receiver_thread_.join(); // 等待线程退出
    }
    depthpub_.reset();
    heightpub_.reset();
    RCUTILS_LOG_INFO_NAMED(
            get_name(),
            "depth on shutdown is called from state %s.",
            state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv); // 初始化ROS 2
    rclcpp::executors::SingleThreadedExecutor exe; // 创建单线程执行器
    std::shared_ptr<Va500pNode> lc_node =
            std::make_shared<Va500pNode>("va500pnode"); // 创建LifecycleTalker节点

    exe.add_node(lc_node->get_node_base_interface()  ); // 将节点添加到执行器
    exe.spin(); // 开始执行循环
    rclcpp::shutdown();

    return 0;
}
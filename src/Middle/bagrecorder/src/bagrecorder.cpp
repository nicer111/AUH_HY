#include "bagrecorder/bagrecorder.h"
#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;
std::shared_ptr<bagrecorderNode> g_node = nullptr;
bagrecorderNode::bagrecorderNode(
        bool record_all,
        const std::map<std::string, bool>& topics_to_record,
        const std::unordered_set<std::string>& video_topics)
        : Node("bagrecorderNode"),
          record_all_(record_all),
          topics_config_(topics_to_record),
          video_topics_(video_topics)
{
    // 初始化存储
    initialize_storage();

    // 创建记录控制服务
    record_control_service_ = create_service<std_srvs::srv::SetBool>(
            "video_record_control",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                handle_record_control(req, res);
            });

    // 启动话题监控（2Hz检查频率）
    topic_monitor_timer_ = create_wall_timer(
            500ms,
            [this]() {
                update_subscriptions();
            });
}

void bagrecorderNode::initialize_storage() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm;
    localtime_r(&now_time_t, &now_tm);

    std::stringstream ss;
    ss << "./bagdate/" << std::put_time(&now_tm, "%Y%m%d%H%M%S") << "_bag";

    storage_options_.storage_id = "mcap";
    storage_options_.uri = ss.str();
    storage_options_.max_bagfile_size = 5242880; // 5MB分块

    // MCAP专用参数（网页2/3的抗断电配置）
    storage_options_.custom_data["mcap.chunk_size"] = "5242880";
    storage_options_.custom_data["mcap.compression"] = "Zstd";
    storage_options_.custom_data["mcap.crc"] = "true";
    storage_options_.custom_data["mcap.sync_interval"] = "0"; // 每次写操作后同步

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    try {
        writer_->open(storage_options_, rosbag2_cpp::ConverterOptions());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "存储初始化失败: %s", e.what());
        rclcpp::shutdown();
    }
    setup_callbacks();
}

void bagrecorderNode::update_subscriptions() {
    std::lock_guard<std::mutex> sub_lock(subscription_mutex_);
    std::lock_guard<std::mutex> ctrl_lock(control_mutex_);

    auto current_topics = get_topic_names_and_types();

    for (const auto& [topic, types] : current_topics) {
        if (active_topics_.count(topic)) continue;

        // 判断是否为视频话题
        bool is_video_topic = video_topics_.count(topic) > 0;

        // 确定记录策略
        bool should_record = false;
        if (record_all_) {
            should_record = true;
        } else if (topics_config_.count(topic)) {
            should_record = topics_config_.at(topic);
        }

        // 视频话题特殊处理
        if (is_video_topic) {
            should_record = should_record && record_video_.load();
        }

        if (should_record) {
            subscribe_to_topic(topic, types.front());
            active_topics_.insert(topic);
            RCLCPP_INFO(get_logger(), "开始记录: %s", topic.c_str());
        }
    }
}

void bagrecorderNode::subscribe_to_topic(const std::string& topic, const std::string& type) {
    try {
        rosbag2_storage::TopicMetadata metadata;
        metadata.name = topic;
        metadata.type = type;
        metadata.serialization_format = "cdr";  // 或使用 rmw_get_serialization_format()
        writer_->create_topic(metadata);

        auto subscription = create_generic_subscription(
                topic,
                type,
                rclcpp::SensorDataQoS(),
                [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    writer_->write(msg, topic, type, now());
                });

        generic_subscriptions_.push_back(subscription);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "订阅失败 %s: %s", topic.c_str(), e.what());
    }
}

void bagrecorderNode::handle_record_control(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
{
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (request->data) {
        record_video_.store(true);
        response->message = "视频记录已启用";
    } else {
        record_video_.store(false);
        response->message = "视频记录已禁用";

        // 取消视频话题订阅
        std::lock_guard<std::mutex> sub_lock(subscription_mutex_);
        for (auto it = generic_subscriptions_.begin(); it != generic_subscriptions_.end();) {
            std::string topic_name = (*it)->get_topic_name(); // 转换为 std::string
            if (video_topics_.count(topic_name)) {
                RCLCPP_INFO(get_logger(), "停止记录视频话题: %s", topic_name.c_str());
                it = generic_subscriptions_.erase(it);
                active_topics_.erase(topic_name);
            } else {
                ++it;
            }
        }
    }

    response->success = true;
}

bagrecorderNode::~bagrecorderNode() {
    if (writer_) {
        writer_->close();
    }
    RCLCPP_INFO(get_logger(), "记录器已安全关闭");
}

void signal_handler(int) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "捕获终止信号，执行优雅关闭");
        g_node->~bagrecorderNode(); // 显式调用析构关闭writer
    }
    rclcpp::shutdown();
}

// main.cpp
int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);

    // 加载配置
    const std::string config_path = "/root/WorkSpace/AUH_WS/src/Middle/bagrecorder/config/config.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    // 基础配置
    bool record_all = config["record_all"].as<bool>();
    std::map<std::string, bool> topics_config;
    if (!record_all) {
        for (const auto& item : config["topics_to_record"]) {
            topics_config[item.first.as<std::string>()] = item.second.as<bool>();
        }
    }

    // 视频话题配置
    std::unordered_set<std::string> video_topics;
    for (const auto& topic : config["video_topics"]) {
        video_topics.insert(topic.as<std::string>());
    }

    g_node = std::make_shared<bagrecorderNode>(record_all, topics_config, video_topics);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}
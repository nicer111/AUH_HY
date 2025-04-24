/********************************************************************************
* @author: Hu Xuanshuo
* @email: huxuanshuo2022@163.com
* @date: 25-2-17 上午11:07
* @version: 2.0
* @description:
********************************************************************************/
#ifndef AUH_WS_BAGRECORDER_H
#define AUH_WS_BAGRECORDER_H

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/bag_events.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <map>
#include <unordered_set>
#include <mutex>
#include <csignal>


class bagrecorderNode : public rclcpp::Node {
public:
    bagrecorderNode(bool record_all,
                    const std::map<std::string, bool>& topics_to_record,
                    const std::unordered_set<std::string>& video_topics);
    virtual ~bagrecorderNode();

private:
    void on_split(rosbag2_cpp::bag_events::BagSplitInfo& info) {
        RCLCPP_INFO(get_logger(),
                    "Bag分块: 已关闭文件=%s, 新文件=%s",
                    info.closed_file.c_str(),
                    info.opened_file.c_str());
    }

    // 注册回调到Writer的正确方式
    void setup_callbacks() {
        rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
        callbacks.write_split_callback = [this](auto& info) {
            this->on_split(info);
        };
        writer_->add_event_callbacks(callbacks);  // 使用官方API
    }

private:
    void initialize_storage();
    void update_subscriptions();
    void subscribe_to_topic(const std::string& topic, const std::string& type);

    // 服务回调
    void handle_record_control(
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response);

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rosbag2_storage::StorageOptions storage_options_;


    // 控制参数
    std::atomic<bool> record_video_{false};
    std::mutex control_mutex_;

    // 话题分类
    const bool record_all_;
    const std::map<std::string, bool> topics_config_;
    const std::unordered_set<std::string> video_topics_;

    // 订阅管理
    rclcpp::TimerBase::SharedPtr topic_monitor_timer_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
    std::unordered_set<std::string> active_topics_;
    std::mutex subscription_mutex_;

    // 服务接口
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_control_service_;
};



#endif //AUH_WS_BAGRECORDER_H


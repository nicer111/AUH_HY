#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <fstream>   // 用于文件操作 (ifstream)
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
class LifecycleServiceClient : public rclcpp::Node {
public:
    explicit LifecycleServiceClient(const std::string & node_name) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "LifecycleServiceClient constructor is called");
        subnodemanage_ = this->create_subscription<std_msgs::msg::String>("changenodestatus", 10,
                                                                          std::bind(&LifecycleServiceClient::nodechange_callback, this, _1));
    }

    void init() {
        // client_get_state_topic 为 lifecycle_talker/get_state
        // client_change_state_topic 为 lifecycle_talker/change_state
        // 上游 lifecycle 节点名为 lifecycle_talker，因此他一定会附赠 lifecycle_talker/get_state 和 lifecycle_talker/change_state service
        // 触发器通过这两个 service ，可以驱动 lifecycle_talker 节点的状态变化
        //client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(client_get_state_topic);
        //client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(client_change_state_topic);
        std::string config_path = "./src/App/nodemanage/config/nodemanage_params.yaml";  // 文件路径

        std::ifstream file(config_path);

        if (!file.good()) {
            std::cerr << "Error: YAML config file not found: " << config_path << std::endl;
        }

        YAML::Node config = YAML::LoadFile(config_path);


        // 获取YAML中的配置
        for (const auto& node : config["nodemanage"]["ros__parameters"]["node_to_manage"]) {
            std::string node_name = node.as<std::string>();
            nodename_to_record_.push_back(node_name);
            std::string change_state_topic = "/AUH/" + node_name + "/change_state";
            // 将节点名称和对应的客户端存入 unordered_map
            client_change_state_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_topic);
            RCLCPP_INFO(this->get_logger(), "Created client for node: %s", node_name.c_str());
        }
        for(const auto & a:nodename_to_record_)
        {
            RCLCPP_INFO(this->get_logger(),"NODENAME: %s",a.c_str());
        }



    }

    //修改指定生命周期节点到指定状态
    void change_node_state(std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client, const std::string & command) {
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_req =
                std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        if (command == "configure") {
            change_state_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        } else if (command == "activate") {
            change_state_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        } else if (command == "deactivate") {
            change_state_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        } else if (command == "cleanup") {
            change_state_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
        } else if(command == "shutdown") {
            change_state_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
        }else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
            return;
        }
        auto future_result = client->async_send_request(change_state_req);
    }

    //处理收到的生命周期节点修改指令
    void handle_command(std::string nodename,std::string command) {
        RCLCPP_INFO(this->get_logger(), "Received command for node: %s, command: %s", nodename.c_str(), command.c_str());

        // 查找对应的客户端
        auto it = client_change_state_.find(nodename);
        if (it != client_change_state_.end()) {
            auto client = it->second; // 获取对应的客户端
            // 根据 command 执行生命周期状态转换
            change_node_state(client, command);
        } else {
            RCLCPP_WARN(this->get_logger(), "No client found for node: %s", nodename.c_str());
        }
    }

    //subnodemanage_这个话题收到发布后会触发这个回调解析消息
    void nodechange_callback(const std_msgs::msg::String & msg)
    {
        //RCLCPP_INFO(this->get_logger(), "get: %s",msg.data.c_str())/
        std::istringstream in(msg.data);
        std::vector<std::string> v;
        std::string t;
        while (in >> t) {
            v.push_back(t);
        }
        handle_command(v.at(0),v.at(1));
    }

private:

    std::vector<std::string> nodename_to_record_;   //从YAML中读取到的节点名，会存在这里面
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> client_change_state_;   //专门用来修改生命周期节点的状态
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subnodemanage_;   //订阅一个话题，专门用来修改指定节点为指定状态

};

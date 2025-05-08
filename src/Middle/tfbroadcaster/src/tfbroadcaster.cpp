#include <cstdio>
#include "tfbroadcaster/tfbroadcaster.h"
TfbroadcasterNode::TfbroadcasterNode()
        : Node("tfbroadcasternode")
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 订阅XY平面坐标
    xysub_ = create_subscription<COORDINATE>(
            "/AUH/coordinate", 10,
            [this](const COORDINATE::SharedPtr msg) {
                current_x_ = msg->x;
                current_y_ = msg->y;
            });
    depthsub_ = create_subscription<std_msgs::msg::Float64>(
            "/AUH/depth", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                current_z_ = msg->data;
            });
    navisub_ = create_subscription<GHFPD>(
            "/AUH/GHFPD", 10,
            [this](const GHFPD::SharedPtr msg) {
                current_roll_ = msg->roll;
                current_pitch_ = msg->pitch;
                current_yaw_ = msg->yaw;
            });
    timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { publish_transform(); });

}

void TfbroadcasterNode::publish_transform() {
    geometry_msgs::msg::TransformStamped transform;

    // 设置时间戳和坐标系关系
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";      // 父坐标系
    transform.child_frame_id = "base_link"; // 子坐标系

    // 设置平移量（来自DVL和深度计）
    transform.transform.translation.x = current_x_;
    transform.transform.translation.y = current_y_;
    transform.transform.translation.z = current_z_;

    // 将欧拉角转换为四元数（来自IMU）
    tf2::Quaternion q;
    q.setRPY(current_roll_, current_pitch_, current_yaw_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // 发布变换
    tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfbroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

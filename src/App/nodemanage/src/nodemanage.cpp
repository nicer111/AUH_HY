#include "nodemanage/nodemanage.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<LifecycleServiceClient> lc_client = std::make_shared<LifecycleServiceClient>("nodemanage");
    lc_client->init();
    rclcpp::spin(lc_client);
    rclcpp::shutdown();
    return 0;
}

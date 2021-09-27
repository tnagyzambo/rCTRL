#include <RocketDataNode.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto rocketDataNode = std::make_shared<RocketDataNode>();
    executor.add_node(rocketDataNode);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

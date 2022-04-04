#include <node.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto rocketGpioNode = std::make_shared<rgpio::Node>();
    executor.add_node(rocketGpioNode);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

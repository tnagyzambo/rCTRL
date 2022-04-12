#include <node.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto rttyNode = std::make_shared<rtty::Node>();

    executor.add_node(rttyNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

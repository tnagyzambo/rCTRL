#include <node.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto rstateNode = std::make_shared<rstate::Node>();

    executor.add_node(rstateNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

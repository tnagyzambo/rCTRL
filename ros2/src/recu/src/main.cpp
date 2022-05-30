#include <node.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto recuNode = std::make_shared<recu::Node>();

    executor.add_node(recuNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

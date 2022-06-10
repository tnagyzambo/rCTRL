#include <node.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto rgpioExampleNode = std::make_shared<recu::Node>();

    executor.add_node(rgpioExampleNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

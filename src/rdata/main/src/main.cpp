#include <RDataNode.hpp>

// The rocketDATA node listens to the topics found in '../msg'. Each topic has its own multi threaded listener.
// This necessitates the use of the multi threaded executor.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto rdataNode = std::make_shared<rdata::Node>();

    executor.add_node(rdataNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

#include <RocketDataNode.hpp>

// The rocketDATA node listens to the topics found in '../msg'. Each topic has its own multi threaded listener.
// This necessitates the use of the multi threaded executor.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto rocketDataNode = std::make_shared<RocketDataNode>();
    executor.add_node(rocketDataNode);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}

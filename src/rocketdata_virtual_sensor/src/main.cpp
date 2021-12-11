#include <VirtualSensorNode.hpp>

int main(int argc, char *argv[])
{
    //auto period = 500ms;

    // Multi threaded executor to allow for potentially different sampling periods
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;

    // auto virtualSensorBool = std::make_shared<VirtualSensorNodeBool>("virtualBool", period);
    // auto virtualSensorFloat64 = std::make_shared<VirtualSensorNodeFloat64>("virtualFloat64", period);
    // auto virtualSensorInt64 = std::make_shared<VirtualSensorNodeInt64>("virtualInt64", period);
    // auto virtualSensorString = std::make_shared<VirtualSensorNodeString>("virtualString", period);
    // auto virtualSensorUint64 = std::make_shared<VirtualSensorNodeUInt64>("virtualUInt64", period);

    // executor.add_node(virtualSensorBool);
    // executor.add_node(virtualSensorFloat64);
    // executor.add_node(virtualSensorInt64);
    // executor.add_node(virtualSensorString);
    // executor.add_node(virtualSensorUint64);
    // executor.spin();

    rclcpp::shutdown();

    return 0;
}
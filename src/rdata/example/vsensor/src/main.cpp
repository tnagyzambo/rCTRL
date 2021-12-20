#include <VSensorNode.hpp>

int main(int argc, char *argv[])
{
    auto period = 500ms;

    // Multi threaded executor to allow for potentially different sampling periods
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // auto vBool = std::make_shared<VirtualSensorNodeBool>("vBool", period);
    auto vF64 = std::make_shared<rdata::vsensor::F64>("vF64", period);
    // auto vI64 = std::make_shared<VirtualSensorNodeInt64>("vI64", period);
    // auto vStr = std::make_shared<VirtualSensorNodeString>("vStr", period);
    // auto vU64 = std::make_shared<VirtualSensorNodeUInt64>("vU64", period);

    // executor.add_node(vBool);
    executor.add_node(vF64);
    // executor.add_node(vI64);
    // executor.add_node(vStr);
    // executor.add_node(vU64);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
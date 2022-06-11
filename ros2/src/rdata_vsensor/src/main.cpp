#include <node.hpp>

int main(int argc, char *argv[]) {
    // Multi threaded executor to allow for potentially different sampling periods
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto vBool = std::make_shared<rdata::vsensor::Bool>("vBool", 100ms, 1000ms);
    auto vF64 = std::make_shared<rdata::vsensor::F64>("vF64", 1ms, 1000ms, 200ms);
    auto vI64 = std::make_shared<rdata::vsensor::I64>("vI64", 100ms, 1000ms);
    auto vStr = std::make_shared<rdata::vsensor::Str>("vStr", 100ms, 1000ms);
    auto vU64 = std::make_shared<rdata::vsensor::U64>("vU64", 100ms, 1000ms);

    executor.add_node(vBool->get_node_base_interface());
    executor.add_node(vF64->get_node_base_interface());
    executor.add_node(vI64->get_node_base_interface());
    executor.add_node(vStr->get_node_base_interface());
    executor.add_node(vU64->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

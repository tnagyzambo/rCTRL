#include <vsensor_node.hpp>

int main(int argc, char *argv[]) {
    auto period = 500ms;

    // Multi threaded executor to allow for potentially different sampling periods
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto vBool = std::make_shared<rdata::vsensor::Bool>("vBool", period);
    auto vF64 = std::make_shared<rdata::vsensor::F64>("vF64", period);
    auto vI64 = std::make_shared<rdata::vsensor::I64>("vI64", period);
    auto vStr = std::make_shared<rdata::vsensor::Str>("vStr", period);
    auto vU64 = std::make_shared<rdata::vsensor::U64>("vU64", period);

    executor.add_node(vBool->get_node_base_interface());
    executor.add_node(vF64->get_node_base_interface());
    executor.add_node(vI64->get_node_base_interface());
    executor.add_node(vStr->get_node_base_interface());
    executor.add_node(vU64->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
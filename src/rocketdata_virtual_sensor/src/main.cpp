#include <VirtualSensorNode.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSensorNode>("test", 4));
    rclcpp::shutdown();

    return 0;
}
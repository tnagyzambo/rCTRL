#include <VirtualSensorNode.hpp>

VirtualSensorNode::VirtualSensorNode(std::string nodeName, _Float64 period) : Node(nodeName), count(0) {
    this->nodeName = nodeName;
    this->period = period;

    this->publisher = this->create_publisher<std_msgs::msg::String>(ROS_ROCKEDATA_TOPIC, 10);
    this->timer = this->create_wall_timer(500ms, std::bind(&VirtualSensorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Creating virtual sensor \"'%s'\" with period \"'%s'\".", this->nodeName.c_str(), std::to_string(this->period).c_str());
}

VirtualSensorNode::~VirtualSensorNode() {
    //RCLCPP_INFO(this->get_logger(), "Destroying virtual sensor \"'%s'\" with period \"'%s'\".", this->nodeName, this->period);
}

void VirtualSensorNode::timerCallback() {
    auto message = std_msgs::msg::String();
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count();
    message.data = std::to_string(sin(mseconds));
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
}
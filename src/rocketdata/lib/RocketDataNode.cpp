#include "RocketDataNode.hpp"

RocketDataNode::RocketDataNode() : rclcpp::Node("rocketdata") {
    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_SUBSCRIPTION_TOPIC << "\".\033[0m" << std::endl;
    std::cout << std::endl;

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        ROS_SUBSCRIPTION_TOPIC, 10, std::bind(&RocketDataNode::topicCallback, this, std::placeholders::_1));
}

RocketDataNode::~RocketDataNode() {
    
}

void RocketDataNode::topicCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std::string test = "test";
    std::string help = msg->data.c_str();

    this->influxClient.writeToInflux(test, test, help);
}
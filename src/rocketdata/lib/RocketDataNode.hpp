#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "InfluxClient.hpp"

#define ROS_SUBSCRIPTION_TOPIC "rocketDATA"

class RocketDataNode : public rclcpp::Node {
    public:
        RocketDataNode();
        ~RocketDataNode();

    private:
        InfluxClient influxClient;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::string readBuffer;

        void topicCallback(const std_msgs::msg::String::SharedPtr);
};
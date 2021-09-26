#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rocketdata/msg/log_bool.hpp>

#define ROS_ROCKEDATA_TOPIC "rocketDATA"

using namespace std::chrono_literals;

class VirtualSensorNode : public rclcpp::Node
{
    public:
        std::string nodeName;
        _Float64 period;

        VirtualSensorNode(std::string, _Float64);
        ~VirtualSensorNode();

    private:
        void timerCallback();

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        size_t count;
};
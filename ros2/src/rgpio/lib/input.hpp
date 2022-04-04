#pragma once

#include <helper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <rgpio_msgs/srv/input_read.hpp>

#define ROS_ROCKEDATA_TOPIC_LOGSTRING "rocketDATA_LogString"

// Input class templated to be either a real or virtual input
namespace rgpio {
    namespace gpio {
        template <class T>
        class Input : public T {
        public:
            Input(rclcpp::Node &, std::string, chip_number, line_number);

            std::string name;

        private:
            rclcpp::Node &node;

            std::string serviceNameRead;

            // rclcpp::Publisher<rgpio_msgs::msg::LogString>::SharedPtr rDataPublisher;

            rclcpp::Service<rgpio_msgs::srv::InputRead>::SharedPtr service;

            std::string createServiceNameRead();

            void read(const std::shared_ptr<rgpio_msgs::srv::InputRead::Request>,
                      std::shared_ptr<rgpio_msgs::srv::InputRead::Response>);
        };

#include <input.tpp>
    } // namespace gpio
} // namespace rgpio

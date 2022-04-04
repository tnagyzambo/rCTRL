#pragma once

#include <helper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <rgpio_msgs/srv/output_write.hpp>

#define ROS_ROCKEDATA_TOPIC_LOGSTRING "rocketDATA_LogString"

namespace rgpio {
    namespace gpio {
        // Output class templated to be either a real or virtual output
        template <class T>
        class Output : public T {
        public:
            Output(rclcpp::Node &, std::string, chip_number, line_number);

            std::string name;

        private:
            rclcpp::Node &node;

            std::string serviceNameWrite;

            // rclcpp::Publisher<rocketgpio::msg::LogString>::SharedPtr rDataPublisher;

            rclcpp::Service<rgpio_msgs::srv::OutputWrite>::SharedPtr service;

            std::string createServiceNameWrite();

            void write(const std::shared_ptr<rgpio_msgs::srv::OutputWrite::Request>,
                       std::shared_ptr<rgpio_msgs::srv::OutputWrite::Response>);
        };

#include <output.tpp>
    } // namespace gpio
} // namespace rgpio

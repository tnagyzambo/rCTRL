#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rgpio_msgs/msg/sim_input.hpp>
#include <rgpio_msgs/msg/sim_output.hpp>

// This class implements all functions needed for accessing virtual IOs, all
// virtual IOs will inherit this class
namespace rgpio {
    namespace gpio {
        class Virtual : public Gpio {
        public:
            Virtual(rclcpp::Node *, std::string, chip_number, line_number);
            ~Virtual();

            void setLineAsInput();
            void setLineAsOutput();

            line_level::level readLine();
            void setLine(line_level::level);

        private:
            rclcpp::Node *node;
            std::string name;

            chip_number chipNumber;
            line_number lineNumber;
            line_level::level level;

            std::string simOutTopic;
            std::string simInTopic;

            rclcpp::Publisher<rgpio_msgs::msg::SimOutput>::SharedPtr simOutPublisher;

            rclcpp::CallbackGroup::SharedPtr callbackGroupSimInSubscriber;
            rclcpp::SubscriptionOptions subscriptionOptSimInSubscriber;
            rclcpp::Subscription<rgpio_msgs::msg::SimInput>::SharedPtr simInSubscriber;

            std::string createSimOutTopic();
            std::string createSimInTopic();

            void topicCallbackSimInSubscriber(const rgpio_msgs::msg::SimInput::SharedPtr);
        };
    } // namespace gpio
} // namespace rgpio

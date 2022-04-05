#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rgpio/util/except.hpp>
#include <rgpio/util/util.hpp>
#include <string>

// Input class templated to be either a real or virtual input
namespace rgpio {
    class Input {
    public:
        Input(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);

        std::unique_ptr<gpio::Gpio> gpio;

        rgpio::gpio::line_level::level read();

    private:
        rclcpp::Node *node;

        // rclcpp::Publisher<rgpio_msgs::msg::LogString>::SharedPtr rDataPublisher;
    };
} // namespace rgpio

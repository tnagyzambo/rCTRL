#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rgpio/util/except.hpp>
#include <rgpio/util/util.hpp>
#include <string>

namespace rgpio {
    class Output {
    public:
        Output(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);

        std::unique_ptr<gpio::Gpio> gpio;

        void write(gpio::line_level::level);

    private:
        rclcpp::Node *node;

        // rclcpp::Publisher<rocketgpio::msg::LogString>::SharedPtr rDataPublisher;
    };
} // namespace rgpio

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rgpio/except.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rutil/toml.hpp>
#include <string>
#include <toml++/toml.h>

// Output class templated to be either a real or virtual output
namespace rgpio {
    class Output {
    public:
        Output(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);

        std::unique_ptr<gpio::Gpio> gpio;

        void write(gpio::line_level::level);
        rgpio::gpio::line_level::level getState();

    private:
        rclcpp::Node *node;

        // rclcpp::Publisher<rocketgpio::msg::LogString>::SharedPtr rDataPublisher;

        static std::unique_ptr<rgpio::gpio::Gpio> constructOutput(rclcpp::Node *,
                                                                  ::toml::node_view<::toml::node>,
                                                                  std::string);
    };
} // namespace rgpio

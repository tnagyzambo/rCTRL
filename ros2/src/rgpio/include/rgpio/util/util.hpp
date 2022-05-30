#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rgpio/gpio/real.hpp>
#include <rgpio/gpio/virtual.hpp>
#include <rgpio/util/except.hpp>
#include <rutil/toml.hpp>
#include <toml++/toml.h>

namespace rgpio {
    std::unique_ptr<rgpio::gpio::Gpio> constructInput(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
    std::unique_ptr<rgpio::gpio::Gpio> constructOutput(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
    std::unique_ptr<rgpio::gpio::Gpio> constructGpio(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
} // namespace rgpio

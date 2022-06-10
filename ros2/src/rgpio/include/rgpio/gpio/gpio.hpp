#pragma once

#include <rgpio/gpio/real.hpp>
#include <rgpio/gpio/util.hpp>
#include <rgpio/gpio/virtual.hpp>

namespace rgpio {
    namespace gpio {
        // Pure function constructor of Gpio
        // This is the simplest way I've found to resolve the awkward inheritance problem
        // Input  \ / Real    \
        //         X           > Gpio
        // Output / \ Virtual /
        std::unique_ptr<rgpio::gpio::Gpio> constructGpio(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
    } // namespace gpio
} // namespace rgpio

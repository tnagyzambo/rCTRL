#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rgpio/gpio/gpio.hpp>
#include <rgpio/gpio/real.hpp>
#include <rgpio/gpio/virtual.hpp>
#include <rgpio/util/except.hpp>
#include <toml++/toml.h>

namespace rgpio {
    std::unique_ptr<rgpio::gpio::Gpio> constructInput(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
    std::unique_ptr<rgpio::gpio::Gpio> constructOutput(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);
    std::unique_ptr<rgpio::gpio::Gpio> constructGpio(rclcpp::Node *, ::toml::node_view<::toml::node>, std::string);

    // TOML++ can return multiple types
    // Templated function for toml value access with standardized formating error handling
    template <typename T>
    T getTomlEntryByKey(::toml::node_view<::toml::node> node_view, const char *key) {
        std::optional<T> entry = node_view[key].value<T>();

        try {
            entry.value();
        } catch (const std::bad_optional_access &e) {
            std::stringstream error;

            error << "Unable to parse key in toml section!\n";
            error << "Key: " << key << "\n";
            error << "TOML: " << node_view << "\n";
            error << "Error: " << e.what() << "\n";

            throw rgpio::except::config_parse_error(error.str());
        }

        return entry.value();
    }
} // namespace rgpio

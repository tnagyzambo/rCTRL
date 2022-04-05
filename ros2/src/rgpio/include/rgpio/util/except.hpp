#pragma once

#include <fmt/core.h>
#include <stdexcept>

namespace rgpio::except {
    class config_parse_error : public std::runtime_error {
    public:
        config_parse_error(std::string error) : std::runtime_error(error) {}
    };

    class gpio_error : public std::runtime_error {
    public:
        gpio_error(std::string error) : std::runtime_error(error) {}
    };
} // namespace rgpio::except

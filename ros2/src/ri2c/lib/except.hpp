#pragma once

#include <stdexcept>

namespace ri2c::except {
    class config_parse_error : public std::runtime_error {
    public:
        config_parse_error(std::string error) : std::runtime_error(error) {}
    };

    class i2c_error : public std::runtime_error {
    public:
        i2c_error(std::string error) : std::runtime_error(error) {}
    };
} // namespace ri2c::except

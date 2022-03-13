#pragma once

#include <fmt/core.h>
#include <stdexcept>

namespace rstate::except {
    class config_parse_error : public std::runtime_error {
    public:
        config_parse_error(std::string error) : std::runtime_error(error) {}
    };
} // namespace rstate::except

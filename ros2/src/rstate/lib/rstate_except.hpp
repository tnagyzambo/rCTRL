#pragma once

#include <fmt/core.h>
#include <stdexcept>

namespace rstate::except {
    class config_parse_error : public std::runtime_error {
    public:
        config_parse_error(std::string error) : std::runtime_error(error) {}
    };

    class cmd_service_eror : public std::runtime_error {
    public:
        cmd_service_eror(std::string error) : std::runtime_error(error) {}
    };
} // namespace rstate::except

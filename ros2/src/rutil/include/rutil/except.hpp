#pragma once

#include <fmt/core.h>
#include <stdexcept>

namespace rutil::except {
    class service_error : public std::runtime_error {
    public:
        service_error(const char *serviceName) : std::runtime_error(buildMessage(serviceName)) {}

    private:
        std::string buildMessage(const char *);
    };

    class toml_parse_error : public std::runtime_error {
    public:
        toml_parse_error(std::string error) : std::runtime_error(error) {}
    };
} // namespace rutil::except

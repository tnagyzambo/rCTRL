#pragma once

#include <stdexcept>
#include <fmt/core.h>

namespace rutil::except
{
    class service_error : public std::runtime_error
    {
    public:
        service_error(const char *serviceName) : std::runtime_error(buildMessage(serviceName)) {}

    private:
        std::string buildMessage(const char *serviceName)
        {
            std::string error = fmt::format("Failed to request service '{}'", serviceName);
            
            return error;
        }
    };
}

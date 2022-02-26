#pragma once

#include <stdexcept>
#include <rutil_fmt.hpp>

namespace rutil::except
{
    class service_error : public std::runtime_error
    {
    public:
        service_error(const char *serviceName) : std::runtime_error(buildMessage(serviceName)) {}

    private:
        std::string buildMessage(const char *serviceName)
        {
            std::string error = "Failed to request service: ";
            error.append(serviceName);

            return error;
        }
    };
}

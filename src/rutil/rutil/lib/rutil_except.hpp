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
            std::string error = rctrl::util::fmt::asciiRedBold.data();
            error.append("Failed to request service: ");
            error.append(serviceName);
            error.append(rctrl::util::fmt::asciiResetFmt.data());

            return error;
        }
    };
}
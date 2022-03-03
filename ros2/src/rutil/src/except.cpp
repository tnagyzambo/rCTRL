#include <except.hpp>

std::string rutil::except::service_error::buildMessage(const char *serviceName)
{
    std::string error = fmt::format("Failed to request service '{}'", serviceName);

    return error;
}

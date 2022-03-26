#include <except.hpp>

namespace rutil::except {
    std::string service_error::buildMessage(const char *serviceName) {
        std::string error = fmt::format("Failed to request service '{}'", serviceName);

        return error;
    }
} // namespace rutil::except

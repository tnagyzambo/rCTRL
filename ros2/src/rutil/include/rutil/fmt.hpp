#pragma once

#include <array>
#include <fmt/core.h>
#include <string_view>

// Formatting for ROS2 lifecycle states
namespace rutil::fmt::state {
    std::string unconfigured();
    std::string inactive();
    std::string active();
    std::string finalized();
} // namespace rutil::fmt::state

// Formatting for ROS2 lifecycle transitions
namespace rutil::fmt::transition {
    std::string constructing();
    std::string configuring();
    std::string cleaningUp();
    std::string shuttingDown();
    std::string activating();
    std::string deactivating();
    std::string errorProcessing();
    std::string destructing();
} // namespace rutil::fmt::transition

namespace rutil::fmt::srv {
    std::string created(std::string service);
    std::string removed(std::string service);
} // namespace rutil::fmt::srv

// Formating for ROS2 subscription operations
namespace rutil::fmt::sub {
    std::string created(std::string subscription);
    std::string removed(std::string subscription);
} // namespace rutil::fmt::sub

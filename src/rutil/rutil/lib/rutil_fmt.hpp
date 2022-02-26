#pragma once

#include <array>
#include <string_view>
#include <fmt/core.h>

// Formatting for ROS2 lifecycle states
namespace rctrl::util::fmt::state
{
    inline std::string unconfigured()
    {
        std::string msg = ::fmt::format("Node is 'Unconfigured'");

        return msg;
    }
    inline std::string inactive()
    {
        std::string msg = ::fmt::format("Node is 'Inactive'");

        return msg;
    }
    inline std::string active()
    {
        std::string msg = ::fmt::format("Node is 'Active'");

        return msg;
    }
    inline std::string finalized()
    {
        std::string msg = ::fmt::format("Node is 'Finalized'");

        return msg;
    }
}

// Formatting for ROS2 lifecycle transitions
namespace rctrl::util::fmt::transition
{
    inline std::string constructing()
    {
        std::string msg = ::fmt::format("Node is 'Constructing'");

        return msg;
    }
    inline std::string configuring()
    {
        std::string msg = ::fmt::format("Node is 'Configuring'");

        return msg;
    }
    inline std::string cleaningUp()
    {
        std::string msg = ::fmt::format("Node is 'Cleaning Up'");

        return msg;
    }
    inline std::string shuttingDown()
    {
        std::string msg = ::fmt::format("Node is 'Shutting Down'");

        return msg;
    }
    inline std::string activating()
    {
        std::string msg = ::fmt::format("Node is 'Activating'");

        return msg;
    }
    inline std::string deactivating()
    {
        std::string msg = ::fmt::format("Node is 'Deactivating'");

        return msg;
    }
    inline std::string errorProcessing()
    {
        std::string msg = ::fmt::format("Node is 'Error Processing'");

        return msg;
    }
    inline std::string destructing()
    {
        std::string msg = ::fmt::format("Node is 'Destructing'");

        return msg;
    }
}

namespace rctrl::util::fmt::srv
{
    inline std::string created(std::string service)
    {
        std::string msg = ::fmt::format("Created service '{}'", service);

        return msg;
    }

    inline std::string removed(std::string service)
    {
        std::string msg = ::fmt::format("Removed service '{}'", service);

        return msg;
    }
}

// Formating for ROS2 subscription operations
namespace rctrl::util::fmt::sub
{
    inline std::string created(std::string subscription)
    {
        std::string msg = ::fmt::format("Created subscription '{}'", subscription);

        return msg;
    }

    inline std::string removed(std::string subscription)
    {
        std::string msg = ::fmt::format("Removed subscription '{}'", subscription);

        return msg;
    }
}

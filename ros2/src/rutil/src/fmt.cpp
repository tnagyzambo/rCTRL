#include <fmt.hpp>

// Formatting for ROS2 lifecycle states
namespace rutil::fmt::state
{
    std::string unconfigured()
    {
        std::string msg = ::fmt::format("Node is 'Unconfigured'");

        return msg;
    }
    std::string inactive()
    {
        std::string msg = ::fmt::format("Node is 'Inactive'");

        return msg;
    }
    std::string active()
    {
        std::string msg = ::fmt::format("Node is 'Active'");

        return msg;
    }
    std::string finalized()
    {
        std::string msg = ::fmt::format("Node is 'Finalized'");

        return msg;
    }
}

// Formatting for ROS2 lifecycle transitions
namespace rutil::fmt::transition
{
    std::string constructing()
    {
        std::string msg = ::fmt::format("Node is 'Constructing'");

        return msg;
    }
    std::string configuring()
    {
        std::string msg = ::fmt::format("Node is 'Configuring'");

        return msg;
    }
    std::string cleaningUp()
    {
        std::string msg = ::fmt::format("Node is 'Cleaning Up'");

        return msg;
    }
    std::string shuttingDown()
    {
        std::string msg = ::fmt::format("Node is 'Shutting Down'");

        return msg;
    }
    std::string activating()
    {
        std::string msg = ::fmt::format("Node is 'Activating'");

        return msg;
    }
    std::string deactivating()
    {
        std::string msg = ::fmt::format("Node is 'Deactivating'");

        return msg;
    }
    std::string errorProcessing()
    {
        std::string msg = ::fmt::format("Node is 'Error Processing'");

        return msg;
    }
    std::string destructing()
    {
        std::string msg = ::fmt::format("Node is 'Destructing'");

        return msg;
    }
}

namespace rutil::fmt::srv
{
    std::string created(std::string service)
    {
        std::string msg = ::fmt::format("Created service '{}'", service);

        return msg;
    }

    std::string removed(std::string service)
    {
        std::string msg = ::fmt::format("Removed service '{}'", service);

        return msg;
    }
}

// Formating for ROS2 subscription operations
namespace rutil::fmt::sub
{
    std::string created(std::string subscription)
    {
        std::string msg = ::fmt::format("Created subscription '{}'", subscription);

        return msg;
    }

    std::string removed(std::string subscription)
    {
        std::string msg = ::fmt::format("Removed subscription '{}'", subscription);

        return msg;
    }
}

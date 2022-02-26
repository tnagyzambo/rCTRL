#pragma once

#include <rutil.hpp>

#include <rdata/srv/create_logger.hpp>
#include <rdata/srv/remove_logger.hpp>

using namespace std::chrono_literals;

namespace rdata::iface
{
    static std::string nodeName = "rdata";

    static std::string srv_get_state = "rdata/get_state";
    static std::string srv_change_state = "rdata/change_state";

    static std::string srv_create_logger_bool = "rdata/srv/create_logger_bool";
    static std::string srv_create_logger_f64 = "rdata/srv/create_logger_f64";
    static std::string srv_create_logger_i64 = "rdata/srv/create_logger_i64";
    static std::string srv_create_logger_str = "rdata/srv/create_logger_str";
    static std::string srv_create_logger_u64 = "rdata/srv/create_logger_u64";

    static std::string srv_remove_logger_bool = "rdata/srv/remove_logger_bool";
    static std::string srv_remove_logger_f64 = "rdata/srv/remove_logger_f64";
    static std::string srv_remove_logger_i64 = "rdata/srv/remove_logger_i64";
    static std::string srv_remove_logger_str = "rdata/srv/remove_logger_str";
    static std::string srv_remove_logger_u64 = "rdata/srv/remove_logger_u64";

    inline void createLogger(const char *serviceName,
                             rclcpp::node_interfaces::NodeBaseInterface::SharedPtr callingNode,
                             rclcpp::Client<rdata::srv::CreateLogger>::SharedPtr clCreateLogger,
                             const char *topicName)
    {
        auto request = std::make_shared<rdata::srv::CreateLogger::Request>();
        request->topic = topicName;

        // It is required to properly coordinate the various lifecycle nodes to avoid waiting for a service that does not exist
        // DO NOT USE wait_for_service()
        // It will block all lifecycle transitions of the calling node
        if (!clCreateLogger->service_is_ready())
        {
            throw rutil::except::service_error(serviceName);
        }

        // We give the async_send_request() method a callback that will get executed once the response
        // is received.
        // This way we can return immediately from this method and allow other work to be done by the
        // executor in `spin` while waiting for the response.
        using ServiceResponseFuture = rclcpp::Client<rdata::srv::CreateLogger>::SharedFuture;
        auto responseReceivedCallback = [callingNode](ServiceResponseFuture future)
        {
            // Handle service response here
            // Currently I don't think this service call can fail so this is mostly placeholder
            auto result = future.get();
            return;
        };
        auto futureResult = clCreateLogger->async_send_request(request, responseReceivedCallback);
    }

    inline void removeLogger(const char *serviceName,
                             rclcpp::node_interfaces::NodeBaseInterface::SharedPtr callingNode,
                             rclcpp::Client<rdata::srv::RemoveLogger>::SharedPtr clRemoveLogger,
                             const char *topicName)
    {
        auto request = std::make_shared<rdata::srv::RemoveLogger::Request>();
        request->topic = topicName;

        // It is required to properly coordinate the various lifecycle nodes to avoid waiting for a service that does not exist
        // DO NOT USE wait_for_service()
        // It will block all lifecycle transitions of the calling node
        if (!clRemoveLogger->service_is_ready())
        {
            throw rutil::except::service_error(serviceName);
        }

        using ServiceResponseFuture = rclcpp::Client<rdata::srv::RemoveLogger>::SharedFuture;
        auto responseReceivedCallback = [callingNode](ServiceResponseFuture future)
        {
            auto result = future.get();
            return;
        };
        auto futureResult = clRemoveLogger->async_send_request(request, responseReceivedCallback);
    }
}

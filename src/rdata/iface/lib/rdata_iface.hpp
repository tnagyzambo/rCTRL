#pragma once

#include <rctrl_except.hpp>

#include <rdata/srv/create_logger.hpp>
#include <rdata/srv/remove_logger.hpp>

using namespace std::chrono_literals;

namespace rdata::iface
{
    static constexpr std::string_view nodeName = "rdata";

    static constexpr std::string_view srv_get_state = "rdata/get_state";
    static constexpr std::string_view srv_change_state = "rdata/change_state";

    static constexpr std::string_view srv_create_logger_bool = "rdata/srv/create_logger_bool";
    static constexpr std::string_view srv_create_logger_f64 = "rdata/srv/create_logger_f64";
    static constexpr std::string_view srv_create_logger_i64 = "rdata/srv/create_logger_i64";
    static constexpr std::string_view srv_create_logger_str = "rdata/srv/create_logger_str";
    static constexpr std::string_view srv_create_logger_u64 = "rdata/srv/create_logger_u64";

    static constexpr std::string_view srv_remove_logger_bool = "rdata/srv/remove_logger_bool";
    static constexpr std::string_view srv_remove_logger_f64 = "rdata/srv/remove_logger_f64";
    static constexpr std::string_view srv_remove_logger_i64 = "rdata/srv/remove_logger_i64";
    static constexpr std::string_view srv_remove_logger_str = "rdata/srv/remove_logger_str";
    static constexpr std::string_view srv_remove_logger_u64 = "rdata/srv/remove_logger_u64";

    inline void createLogger(const char *serviceName,
                             rclcpp::node_interfaces::NodeBaseInterface::SharedPtr callingNode,
                             rclcpp::Client<rdata::srv::CreateLogger>::SharedPtr clCreateLogger,
                             const char *topicName)
    {
        auto request = std::make_shared<rdata::srv::CreateLogger::Request>();
        request->topic = topicName;

        while (!clCreateLogger->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                // This will throw an rcl exception
                // Im pretty sure this is a ROS2 issue resulting from a race condition on shutdown
                // Just have to live with this for now, exiting anyway
                // REFERENCE: https://github.com/ros2/rclcpp/issues/1139
                RCLCPP_ERROR(rclcpp::get_logger(callingNode->get_name()), "\033[1;31mInterupted while waiting for '%s'!, exiting\033[0m", serviceName);
                throw rctrl::util::except::service_error(serviceName);
            }
            RCLCPP_INFO(rclcpp::get_logger(callingNode->get_name()), "Service '%s' not available, waiting", serviceName);
        }

        // We give the async_send_request() method a callback that will get executed once the response
        // is received.
        // This way we can return immediately from this method and allow other work to be done by the
        // executor in `spin` while waiting for the response.
        using ServiceResponseFuture = rclcpp::Client<rdata::srv::CreateLogger>::SharedFuture;
        auto responseReceivedCallback = [callingNode](ServiceResponseFuture future)
        {
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

        while (!clRemoveLogger->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger(callingNode->get_name()), "\033[1;31mInterupted while waiting for '%s'!, exiting\033[0m", serviceName);
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger(callingNode->get_name()), "Service '%s' not available, waiting", serviceName);
        }

        using ServiceResponseFuture = rclcpp::Client<rdata::srv::RemoveLogger>::SharedFuture;
        auto responseReceivedCallback = [callingNode](ServiceResponseFuture future)
        {
            auto result = future.get();
            RCLCPP_INFO(rclcpp::get_logger(callingNode->get_name()), "hello");
            return;
        };
        auto futureResult = clRemoveLogger->async_send_request(request, responseReceivedCallback);
    }
}

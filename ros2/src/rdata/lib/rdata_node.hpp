#pragma once

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <influx_client.hpp>
#include <rdata/iface.hpp>
#include <rutil/fmt.hpp>
#include <rutil/except.hpp>

#include <rdata/srv/create_logger.hpp>
#include <rdata/srv/remove_logger.hpp>

#include <rdata/msg/log_bool.hpp>
#include <rdata/msg/log_f64.hpp>
#include <rdata/msg/log_i64.hpp>
#include <rdata/msg/log_str.hpp>
#include <rdata/msg/log_u64.hpp>

namespace rdata
{
    template <typename T>
    struct Logger
    {
        typename rclcpp::Subscription<T>::SharedPtr subPtr;
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        rclcpp::SubscriptionOptions opts;
    };

    class Node : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        Node();
        ~Node();

    private:
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerBool;
        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerF64;
        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerI64;
        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerStr;
        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerU64;

        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerBool;
        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerF64;
        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerI64;
        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerStr;
        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerU64;

        std::vector<rdata::Logger<rdata::msg::LogBool>> loggersBool;
        std::vector<rdata::Logger<rdata::msg::LogF64>> loggersF64;
        std::vector<rdata::Logger<rdata::msg::LogI64>> loggersI64;
        std::vector<rdata::Logger<rdata::msg::LogStr>> loggersStr;
        std::vector<rdata::Logger<rdata::msg::LogU64>> loggersU64;

        void callbackLogBool(const rdata::msg::LogBool::SharedPtr);
        void callbackLogF64(const rdata::msg::LogF64::SharedPtr);
        void callbackLogI64(const rdata::msg::LogI64::SharedPtr);
        void callbackLogStr(const rdata::msg::LogStr::SharedPtr);
        void callbackLogU64(const rdata::msg::LogU64::SharedPtr);

        void createLoggerBool(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                              std::shared_ptr<rdata::srv::CreateLogger::Response>);
        void createLoggerF64(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                             std::shared_ptr<rdata::srv::CreateLogger::Response>);
        void createLoggerI64(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                             std::shared_ptr<rdata::srv::CreateLogger::Response>);
        void createLoggerStr(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                             std::shared_ptr<rdata::srv::CreateLogger::Response>);
        void createLoggerU64(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                             std::shared_ptr<rdata::srv::CreateLogger::Response>);

        void removeLoggerBool(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                              std::shared_ptr<rdata::srv::RemoveLogger::Response>);
        void removeLoggerF64(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                             std::shared_ptr<rdata::srv::RemoveLogger::Response>);
        void removeLoggerI64(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                             std::shared_ptr<rdata::srv::RemoveLogger::Response>);
        void removeLoggerStr(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                             std::shared_ptr<rdata::srv::RemoveLogger::Response>);
        void removeLoggerU64(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                             std::shared_ptr<rdata::srv::RemoveLogger::Response>);

        template <typename T>
        static std::vector<typename rdata::Logger<T>> removeLoggerByTopic(std::vector<typename rdata::Logger<T>>, const char *);

        void deleteAllPointers();

        std::unique_ptr<influx::Client> influxClient;
        std::string readBuffer;

        template <typename T>
        void tryToWriteToInfluxDB(T);
    };

// Implementation of templated functions
#include "rdata_node.tpp"
}

#pragma once

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <InfluxClient.hpp>
#include <RDataIFace.hpp>

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

        rclcpp::Service<rdata::srv::CreateLogger>::SharedPtr srvCreateLoggerF64;

        rclcpp::Service<rdata::srv::RemoveLogger>::SharedPtr srvRemoveLoggerF64;

        std::vector<rdata::Logger<rdata::msg::LogBool>> loggersBool;
        std::vector<rdata::Logger<rdata::msg::LogF64>> loggersF64;
        std::vector<rdata::Logger<rdata::msg::LogI64>> loggersI64;
        std::vector<rdata::Logger<rdata::msg::LogStr>> loggersStr;
        std::vector<rdata::Logger<rdata::msg::LogU64>> loggersU64;

        void callbackLogF64(const rdata::msg::LogF64::SharedPtr);

        void createLoggerF64(const std::shared_ptr<rdata::srv::CreateLogger::Request>,
                             std::shared_ptr<rdata::srv::CreateLogger::Response>);

        void removeLoggerF64(const std::shared_ptr<rdata::srv::RemoveLogger::Request>,
                             std::shared_ptr<rdata::srv::RemoveLogger::Response>);

        template <typename T>
        static std::vector<typename rdata::Logger<T>> removeLoggerByTopic(std::vector<typename rdata::Logger<T>>, const char *);

        void deleteAllPointers();

        influxclient::Client influxClient;
        std::string readBuffer;

        template <typename T>
        void tryToWriteToInfluxDB(T);
    };

// Implementation of templated functions
#include "RDataNode.tpp"
}

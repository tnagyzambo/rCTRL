#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rdata_iface.hpp>
#include <rutil.hpp>

#include <rdata/srv/create_logger.hpp>
#include <rdata/srv/remove_logger.hpp>

#include <rdata/msg/log_bool.hpp>
#include <rdata/msg/log_f64.hpp>
#include <rdata/msg/log_i64.hpp>
#include <rdata/msg/log_str.hpp>
#include <rdata/msg/log_u64.hpp>

using namespace std::chrono_literals;

namespace rdata::vsensor
{
    template <typename T>
    class Node : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        Node(const char *, std::chrono::milliseconds);
        ~Node();

        const char *nodeName;
        std::string loggerTopicName;
        std::chrono::milliseconds period;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Client<rdata::srv::CreateLogger>::SharedPtr clCreateLogger;
        rclcpp::Client<rdata::srv::RemoveLogger>::SharedPtr clRemoveLogger;
        typename rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr logger;

        uint calcElapsedTime();

    private:
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        // Virtual callback function so superclasses can overide implementation
        virtual void timerCallback();
        std::string createLoggerTopicName();
    };

    class Bool : public Node<rdata::msg::LogBool>
    {
    public:
        Bool(const char *, std::chrono::milliseconds);
        ~Bool();

    private:
        bool prevOutput = false;

        void timerCallback();
    };

    class F64 : public Node<rdata::msg::LogF64>
    {
    public:
        F64(const char *, std::chrono::milliseconds);
        ~F64();

    private:
        double sinPeriod;

        void timerCallback();
    };

    class I64 : public Node<rdata::msg::LogI64>
    {
    public:
        I64(const char *, std::chrono::milliseconds);
        ~I64();

    private:
        void timerCallback();
    };

    class Str : public Node<rdata::msg::LogStr>
    {
    public:
        Str(const char *, std::chrono::milliseconds);
        ~Str();

    private:
        void timerCallback();
    };

    class U64 : public Node<rdata::msg::LogU64>
    {
    public:
        U64(const char *, std::chrono::milliseconds);
        ~U64();

    private:
        void timerCallback();
    };

// Implementation of templated functions
#include "vsensor_node.tpp"
}
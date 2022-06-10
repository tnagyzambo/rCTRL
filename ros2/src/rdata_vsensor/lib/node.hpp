#pragma once

#include <chrono>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rdata/logger.hpp>
#include <rutil/except.hpp>
#include <rutil/fmt.hpp>
#include <string>

using namespace std::chrono_literals;

namespace rdata::vsensor {
    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node(const char *, std::chrono::milliseconds, std::chrono::milliseconds);
        ~Node();

        const char *nodeName;
        std::unique_ptr<rdata::Logger> logger;
        std::chrono::milliseconds samplePeriod;
        std::chrono::milliseconds loggingPeriod;

        uint calcElapsedTime();

    private:
        rclcpp::TimerBase::SharedPtr sampleTimer;
        rclcpp::TimerBase::SharedPtr loggerTimer;

        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &state);

        // Virtual callback function so superclasses can overide implementation
        virtual void sampleCallback();
        void loggerCallback();
        std::string createLoggerTopicName();
    };

    class Bool : public Node {
    public:
        Bool(const char *, std::chrono::milliseconds, std::chrono::milliseconds);
        ~Bool();

    private:
        bool prevOutput = false;

        void sampleCallback();
    };

    class F64 : public Node {
    public:
        F64(const char *, std::chrono::milliseconds, std::chrono::milliseconds, std::chrono::milliseconds);
        ~F64();

    private:
        double sinPeriod;

        void sampleCallback();
    };

    class I64 : public Node {
    public:
        I64(const char *, std::chrono::milliseconds, std::chrono::milliseconds);
        ~I64();

    private:
        void sampleCallback();
    };

    class Str : public Node {
    public:
        Str(const char *, std::chrono::milliseconds, std::chrono::milliseconds);
        ~Str();

    private:
        void sampleCallback();
    };

    class U64 : public Node {
    public:
        U64(const char *, std::chrono::milliseconds, std::chrono::milliseconds);
        ~U64();

    private:
        void sampleCallback();
    };
} // namespace rdata::vsensor

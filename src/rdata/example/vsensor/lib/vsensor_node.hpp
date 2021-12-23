#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rdata_iface.hpp>

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
    class Node : public rclcpp::Node
    {
    public:
        Node(const char *, std::chrono::milliseconds);

        const char *nodeName;
        std::string loggerTopicName;
        std::chrono::milliseconds period;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Client<rdata::srv::CreateLogger>::SharedPtr clCreateLogger;
        rclcpp::Client<rdata::srv::RemoveLogger>::SharedPtr clRemoveLogger;
        typename rclcpp::Publisher<T>::SharedPtr logger;

        uint calcElapsedTime();

    private:
        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        // Virtual callback function so superclasses can overide implementation
        virtual void timerCallback();
        std::string createLoggerTopicName();
    };

    // class VirtualSensorNodeBool : public VirtualSensorNode
    // {
    // public:
    //     VirtualSensorNodeBool(std::string, std::chrono::milliseconds);
    //     ~VirtualSensorNodeBool();

    // private:
    //     rclcpp::Publisher<rdata::msg::LogBool>::SharedPtr publisher;
    //     bool prevOutput = false;

    //     void timerCallback();
    // };

    class F64 : public Node<rdata::msg::LogF64>
    {
    public:
        F64(const char *, std::chrono::milliseconds);
        ~F64();

    private:
        double sinPeriod;

        void timerCallback();
    };

// class VirtualSensorNodeInt64 : public VirtualSensorNode
// {
// public:
//     VirtualSensorNodeInt64(std::string, std::chrono::milliseconds);
//     ~VirtualSensorNodeInt64();

// private:
//     rclcpp::Publisher<rdata::msg::LogI64>::SharedPtr publisher;

//     void timerCallback();
// };

// class VirtualSensorNodeString : public VirtualSensorNode
// {
// public:
//     VirtualSensorNodeString(std::string, std::chrono::milliseconds);
//     ~VirtualSensorNodeString();

// private:
//     rclcpp::Publisher<rdata::msg::LogStr>::SharedPtr publisher;

//     void timerCallback();
// };

// class VirtualSensorNodeUInt64 : public VirtualSensorNode
// {
// public:
//     VirtualSensorNodeUInt64(std::string, std::chrono::milliseconds);
//     ~VirtualSensorNodeUInt64();

// private:
//     rclcpp::Publisher<rdata::msg::LogU64>::SharedPtr publisher;

//     void timerCallback();
// };

// Implementation of templated functions
#include "vsensor_node.tpp"
}
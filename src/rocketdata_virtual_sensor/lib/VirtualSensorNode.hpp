#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rocketdata/msg/log_bool.hpp>
#include <rocketdata/msg/log_float64.hpp>
#include <rocketdata/msg/log_int64.hpp>
#include <rocketdata/msg/log_string.hpp>
#include <rocketdata/msg/log_uint64.hpp>

#define ROS_ROCKEDATA_TOPIC_LOGBOOL "rocketDATA_LogBool"
#define ROS_ROCKEDATA_TOPIC_LOGFLOAT64 "rocketDATA_LogFloat64"
#define ROS_ROCKEDATA_TOPIC_LOGINT64 "rocketDATA_LogInt64"
#define ROS_ROCKEDATA_TOPIC_LOGSTRING "rocketDATA_LogString"
#define ROS_ROCKEDATA_TOPIC_LOGUINT64 "rocketDATA_LogUint64"

using namespace std::chrono_literals;

class VirtualSensorNode : public rclcpp::Node
{
public:
    std::string nodeName;
    std::chrono::milliseconds period;

    VirtualSensorNode(std::string, std::chrono::milliseconds);

    uint calcElapsedTime();

private:
    rclcpp::TimerBase::SharedPtr timer;
    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

    // Virtual callback function so superclasses can overide implementation
    virtual void timerCallback();
};

class VirtualSensorNodeBool : public VirtualSensorNode
{
public:
    VirtualSensorNodeBool(std::string, std::chrono::milliseconds);
    ~VirtualSensorNodeBool();

private:
    rclcpp::Publisher<rocketdata::msg::LogBool>::SharedPtr publisher;
    bool prevOutput = false;

    void timerCallback();
};

class VirtualSensorNodeFloat64 : public VirtualSensorNode
{
public:
    VirtualSensorNodeFloat64(std::string, std::chrono::milliseconds);
    ~VirtualSensorNodeFloat64();

private:
    rclcpp::Publisher<rocketdata::msg::LogFloat64>::SharedPtr publisher;
    double sinPeriod;

    void timerCallback();
};

class VirtualSensorNodeInt64 : public VirtualSensorNode
{
public:
    VirtualSensorNodeInt64(std::string, std::chrono::milliseconds);
    ~VirtualSensorNodeInt64();

private:
    rclcpp::Publisher<rocketdata::msg::LogInt64>::SharedPtr publisher;

    void timerCallback();
};

class VirtualSensorNodeString : public VirtualSensorNode
{
public:
    VirtualSensorNodeString(std::string, std::chrono::milliseconds);
    ~VirtualSensorNodeString();

private:
    rclcpp::Publisher<rocketdata::msg::LogString>::SharedPtr publisher;

    void timerCallback();
};

class VirtualSensorNodeUInt64 : public VirtualSensorNode
{
public:
    VirtualSensorNodeUInt64(std::string, std::chrono::milliseconds);
    ~VirtualSensorNodeUInt64();

private:
    rclcpp::Publisher<rocketdata::msg::LogUint64>::SharedPtr publisher;

    void timerCallback();
};
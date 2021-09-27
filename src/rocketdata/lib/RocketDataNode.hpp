#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <InfluxClient.hpp>
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

class RocketDataNode : public rclcpp::Node {
    public:
        RocketDataNode();
        ~RocketDataNode();

    private:
        rclcpp::CallbackGroup::SharedPtr callbackGroupSubcriberLogBool;
        rclcpp::CallbackGroup::SharedPtr callbackGroupSubcriberLogFloat64;
        rclcpp::CallbackGroup::SharedPtr callbackGroupSubcriberLogInt64;
        rclcpp::CallbackGroup::SharedPtr callbackGroupSubcriberLogString;
        rclcpp::CallbackGroup::SharedPtr callbackGroupSubcriberLogUint64;

        rclcpp::SubscriptionOptions subscriptionLogBoolOpt;
        rclcpp::SubscriptionOptions subscriptionLogFloat64Opt;
        rclcpp::SubscriptionOptions subscriptionLogInt64Opt;
        rclcpp::SubscriptionOptions subscriptionLogStringOpt;
        rclcpp::SubscriptionOptions subscriptionLogUint64Opt;

        rclcpp::Subscription<rocketdata::msg::LogBool>::SharedPtr subscriptionLogBool;
        rclcpp::Subscription<rocketdata::msg::LogFloat64>::SharedPtr subscriptionLogFloat64;
        rclcpp::Subscription<rocketdata::msg::LogInt64>::SharedPtr subscriptionLogInt64;
        rclcpp::Subscription<rocketdata::msg::LogString>::SharedPtr subscriptionLogString;
        rclcpp::Subscription<rocketdata::msg::LogUint64>::SharedPtr subscriptionLogUint64;

        InfluxClient influxClient;
        std::string readBuffer;

        void topicCallbackLogBool(const rocketdata::msg::LogBool::SharedPtr);
        void topicCallbackLogFloat64(const rocketdata::msg::LogFloat64::SharedPtr);
        void topicCallbackLogInt64(const rocketdata::msg::LogInt64::SharedPtr);
        void topicCallbackLogString(const rocketdata::msg::LogString::SharedPtr);
        void topicCallbackLogUint64(const rocketdata::msg::LogUint64::SharedPtr);
};

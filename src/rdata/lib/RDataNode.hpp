#pragma once

#include <string>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <InfluxClient.hpp>

#include <rdata/srv/create_sub.hpp>

#include <rdata/srv/remove_sub.hpp>

#include <rdata/msg/log_bool.hpp>
#include <rdata/msg/log_f64.hpp>
#include <rdata/msg/log_i64.hpp>
#include <rdata/msg/log_str.hpp>
#include <rdata/msg/log_u64.hpp>

#define RDATA_SRV_CREATE_SUB_BOOL "rdata_srv_create_sub_bool"

#define RDATA_SRV_REMOVE_SUB_BOOL "rdata_srv_remove_sub_bool"

namespace rdata
{
    template <typename T>
    struct Sub
    {
        typename rclcpp::Subscription<T>::SharedPtr subPtr;
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        rclcpp::SubscriptionOptions opts;
    };

    class Node : public rclcpp::Node
    {
    public:
        Node();
        ~Node();

    private:
        rclcpp::Service<rdata::srv::CreateSub>::SharedPtr srvCreateSubBool;

        rclcpp::Service<rdata::srv::RemoveSub>::SharedPtr srvRemoveSubBool;

        std::vector<rdata::Sub<rdata::msg::LogBool>> subsBool;
        std::vector<rclcpp::Subscription<rdata::msg::LogF64>::SharedPtr> subsF64;
        std::vector<rclcpp::Subscription<rdata::msg::LogI64>::SharedPtr> subsI64;
        std::vector<rclcpp::Subscription<rdata::msg::LogStr>::SharedPtr> subsStr;
        std::vector<rclcpp::Subscription<rdata::msg::LogU64>::SharedPtr> subsU64;

        void callbackLogBool(const rdata::msg::LogBool::SharedPtr);

        void createSubBool(const std::shared_ptr<rdata::srv::CreateSub::Request>,
                           std::shared_ptr<rdata::srv::CreateSub::Response>);

        void removeSubBool(const std::shared_ptr<rdata::srv::RemoveSub::Request>,
                           std::shared_ptr<rdata::srv::RemoveSub::Response>);

        template <typename T>
        static std::vector<typename rdata::Sub<T>> removeSubByTopic(std::vector<typename rdata::Sub<T>> subs, const char *topicName);

        influxclient::Client influxClient;
        std::string readBuffer;

        template <typename T>
        void tryToWriteToInfluxDB(T);
    };

// Implementation of templated functions
#include "RDataNode.tpp"
}

#pragma once

#include <string>
#include <memory>
#include <curl/curl.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "InfluxClient.hpp"

class RosNode : public rclcpp::Node {
    public:
        RosNode();

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr) const;

        static size_t WriteCallback(char*, size_t, size_t, void*);

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        InfluxClient influxClient;
        std::string readBuffer;
        CURL *curl;
        CURLcode res;
        std::string curlReadBuffer;
        struct curl_slist* headers = NULL;
        std::string authorization;
};
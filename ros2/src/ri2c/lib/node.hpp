#pragma once

#include <ads1014.hpp>
#include <cstring>
#include <except.hpp>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rdata/logger.hpp>
#include <rutil/except.hpp>
#include <rutil/fmt.hpp>
#include <rutil/toml.hpp>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

namespace ri2c {
    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

    private:
        int i2cBus;
        std::string i2cBusName;

        std::unique_ptr<ADS1014> sensor1;
        // std::unique_ptr<ADS1014> sensor2;
        // std::unique_ptr<ADS1014> sensor3;
        // std::unique_ptr<ADS1014> sensor4;

        std::unique_ptr<rdata::Logger> loggerLowSpeed;
        std::unique_ptr<rdata::Logger> loggerHighSpeed;

        std::chrono::milliseconds samplePeriodLowSpeed;
        std::chrono::milliseconds loggingPeriodLowSpeed;
        std::chrono::milliseconds samplePeriodHighSpeed;

        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeed;
        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeedWrite;
        rclcpp::TimerBase::SharedPtr timerDataLoggingHighSpeed;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &);

        void callbackDataLoggingLowSpeed();
        void callbackDataLoggingLowSpeedWrite();
        void callbackDataLoggingHighSpeed();

        void readConfig(toml::table);

        void deleteAllPointers();
    };
} // namespace ri2c

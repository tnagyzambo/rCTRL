#pragma once

#include <ads1014.hpp>
#include <chrono>
#include <cstring>
#include <except.hpp>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rdata/logger.hpp>
#include <recu_msgs/srv/valve_action.hpp>
#include <ri2c_msgs/msg/valve_state.hpp>
#include <ri2c_msgs/srv/detail/high_speed_data_logging_action__struct.hpp>
#include <ri2c_msgs/srv/get_valve_state.hpp>
#include <ri2c_msgs/srv/high_speed_data_logging_action.hpp>
#include <ri2c_msgs/srv/pres_control_loop_action.hpp>
#include <rutil/except.hpp>
#include <rutil/fmt.hpp>
#include <rutil/toml.hpp>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
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
        float pressureSetPoint;
        bool presControlState;
        bool lastPressureControlCommand; // True for powered, false for unpowered. Used to avoid sending extra service calls

        std::unique_ptr<PAA_7LC_30BAR> p_h2o2;
        std::unique_ptr<PAA_7LC_30BAR> p_ethanol;
        std::unique_ptr<PAA_7LC_30BAR> p_pressurant;
        std::unique_ptr<LoadcellBridge> loadcell;
        std::unique_ptr<M5HB_30BAR> p_chamber;
        std::unique_ptr<M5HB_30BAR> p_manifold;
        std::unique_ptr<K_TYPE> t_chamber;

        std::unique_ptr<rdata::Logger> loggerLowSpeed;
        std::unique_ptr<rdata::Logger> loggerHighSpeed;

        std::chrono::milliseconds samplePeriodLowSpeed;
        std::chrono::milliseconds loggingPeriodLowSpeed;
        std::chrono::milliseconds samplePeriodHighSpeed;
        std::chrono::milliseconds pressureControlLoopRate;

        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeed;
        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeedWrite;
        rclcpp::TimerBase::SharedPtr timerDataLoggingHighSpeed;
        rclcpp::TimerBase::SharedPtr timerPressureControlLoop;

        rclcpp::Service<ri2c_msgs::srv::HighSpeedDataLoggingAction>::SharedPtr srvDataLoggingHighSpeedOn;
        rclcpp::Service<ri2c_msgs::srv::HighSpeedDataLoggingAction>::SharedPtr srvDataLoggingHighSpeedOff;
        rclcpp::Service<ri2c_msgs::srv::PresControlLoopAction>::SharedPtr srvPresControlOn;
        rclcpp::Service<ri2c_msgs::srv::PresControlLoopAction>::SharedPtr srvPresControlOff;
        rclcpp::Service<ri2c_msgs::srv::GetValveState>::SharedPtr srvPresControlGetState;

        rclcpp::Client<recu_msgs::srv::ValveAction>::SharedPtr clPV_Open;
        rclcpp::Client<recu_msgs::srv::ValveAction>::SharedPtr clPV_Close;

        void callbackDataLoggingHighSpeedOn(const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request>,
                                            std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response>);
        void callbackDataLoggingHighSpeedOff(const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request>,
                                             std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response>);

        void callbackPresControlOn(const std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Request>,
                                   std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Response>);
        void callbackPresControlOff(const std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Request>,
                                    std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Response>);

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

        ri2c_msgs::srv::GetValveState::Response createValveStateResponse(bool);

        void callbackDataLoggingLowSpeed();
        void callbackDataLoggingLowSpeedWrite();
        void callbackDataLoggingHighSpeed();
        void callbackPressureControlLoop();
        void PresControl_GetState(const std::shared_ptr<ri2c_msgs::srv::GetValveState::Request>,
                                  std::shared_ptr<ri2c_msgs::srv::GetValveState::Response>);

        void readConfig(toml::table);

        void deleteAllPointers();
    };
} // namespace ri2c

#pragma once

#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <recu_msgs/msg/valve_state.hpp>
#include <recu_msgs/srv/arduino_action.hpp>
#include <recu_msgs/srv/get_valve_state.hpp>
#include <rutil/fmt.hpp>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

// REFERENCE: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

namespace rtty {
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    enum ValveState {
        Closed,
        Open,
        Unknown,
    };

    enum EcuActions {
        Nothing,
        MV1_Open,
        MV1_Close,
        MV2_Open,
        MV2_Close,
        PV_Open,
        PV_Close,
        ESV_Open,
        ESV_Close,
    };

    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

    private:
        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        void deleteAllPointers();

        void serialClose();
        void serialRead();
        void serialHandleMsg();
        void serialWrite(EcuActions);

        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV1_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV1_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV1_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV2_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvPV_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvESV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvESV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvESV_GetState;

        void MV1_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV1_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV1_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void MV2_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV2_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV2_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void PV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                     std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void PV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void PV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                         std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void ESV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void ESV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void ESV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);

        rclcpp::TimerBase::SharedPtr read_timer;
        int serial_port;
        char read_buf[1024];
        uint read_buf_l;
    };
} // namespace rtty

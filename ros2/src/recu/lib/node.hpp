#pragma once

#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rdata/iface.hpp>
#include <rdata_msgs/msg/log_f64.hpp>
#include <rdata_msgs/srv/create_logger.hpp>
#include <rdata_msgs/srv/remove_logger.hpp>
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
using json = nlohmann::json;

// REFERENCE: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

namespace recu {
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    struct JsonData {
        bool stateMV1;
        bool stateMV2;
        bool statePV;
        bool stateBV;
        float loadCell;
        float pressureTank1;
        float pressureTank2;
        float pressureChamber1;
        float pressureChamber2;
        float tempThermocouple1;
        float tempThermocouple2;
        float tempThermocouple3;
    };

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
        BV_Open,
        BV_Close,
        Fire,
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

        ValveState stateMV1 = Unknown;
        ValveState stateMV2 = Unknown;
        ValveState statePV = Unknown;
        ValveState stateBV = Unknown;

        rclcpp::Client<rdata_msgs::srv::CreateLogger>::SharedPtr clCreateLogger;
        rclcpp::Client<rdata_msgs::srv::RemoveLogger>::SharedPtr clRemoveLogger;
        rclcpp_lifecycle::LifecyclePublisher<rdata_msgs::msg::LogF64>::SharedPtr logger;

        recu_msgs::srv::GetValveState::Response createValveStateResponse(ValveState);

        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV1_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV1_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV1_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV2_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvPV_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvBV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvBV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvBV_GetState;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvFire;

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
        void BV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                     std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void BV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void BV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                         std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void Fire(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                  std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);

        rclcpp::TimerBase::SharedPtr read_timer;
        int serial_port;
        char read_buf[1024];
        uint read_buf_l = 0;
    };
} // namespace recu

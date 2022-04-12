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
#include <recu_msgs/srv/arduino_action.hpp>
#include <recu_msgs/srv/detail/arduino_action__struct.hpp>
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
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvMV2_Close;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvPV_Close;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvESV_Open;
        rclcpp::Service<recu_msgs::srv::ArduinoAction>::SharedPtr srvESV_Close;

        void MV1_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV1_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV2_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void MV2_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void PV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                     std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void PV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void ESV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);
        void ESV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response>);

        rclcpp::TimerBase::SharedPtr read_timer;
        int serial_port;
        char read_buf[1024];
        uint read_buf_l;
    };
} // namespace rtty

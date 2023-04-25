#pragma once

#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rdata/logger.hpp>
#include <recu_msgs/msg/valve_state.hpp>
#include <recu_msgs/srv/get_valve_state.hpp>
#include <recu_msgs/srv/valve_action.hpp>
#include <rgpio/input.hpp>
#include <rgpio/output.hpp>
#include <ri2c_msgs/srv/detail/pres_control_loop_action__struct.hpp>
#include <ri2c_msgs/srv/high_speed_data_logging_action.hpp>
#include <rutil/fmt.hpp>
#include <rutil/toml.hpp>
#include <string>

using namespace std::chrono_literals;

namespace recu {
    enum ValveState {
        Closed,
        Open,
        Unknown,
    };

    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

    private:
        // This is a horrible, very scary line but it should work for what we need to do
        // https://stackoverflow.com/questions/21531096/can-i-use-stdasync-without-waiting-for-the-future-limitation
        std::vector<rclcpp::Client<ri2c_msgs::srv::HighSpeedDataLoggingAction>::FutureAndRequestId> pending_futures;
        std::vector<rclcpp::Client<ri2c_msgs::srv::PresControlLoopAction>::FutureAndRequestId> pending_futures_pres;

        std::unique_ptr<rgpio::Output> valveBV;
        std::unique_ptr<rgpio::Output> valvePV;
        std::unique_ptr<rgpio::Output> valveMV1;
        std::unique_ptr<rgpio::Output> valveMV2;
        std::unique_ptr<rgpio::Output> valvePurge;
        std::unique_ptr<rgpio::Output> pyro;

        rclcpp::TimerBase::SharedPtr ignitionSequenceTimer;
        std::chrono::high_resolution_clock::time_point ignitionSequenceStartTime;

        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvIgnitionSequence;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvAbort;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvBV_Open;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvBV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvBV_GetState;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvPV_Open;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvPV_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvPV_GetState;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvMV1_Open;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvMV1_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV1_GetState;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvMV2_Open;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvMV2_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvMV2_GetState;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvPurge_Open;
        rclcpp::Service<recu_msgs::srv::ValveAction>::SharedPtr srvPurge_Close;
        rclcpp::Service<recu_msgs::srv::GetValveState>::SharedPtr srvPurge_GetState;

        rclcpp::Client<ri2c_msgs::srv::HighSpeedDataLoggingAction>::SharedPtr clHighSpeedDataLoggingOn;
        rclcpp::Client<ri2c_msgs::srv::HighSpeedDataLoggingAction>::SharedPtr clHighSpeedDataLoggingOff;
        rclcpp::Client<ri2c_msgs::srv::PresControlLoopAction>::SharedPtr clPresControlLoopOn;
        rclcpp::Client<ri2c_msgs::srv::PresControlLoopAction>::SharedPtr clPresControlLoopOff;

        std::unique_ptr<rdata::Logger> loggerLowSpeed;
        std::unique_ptr<rdata::Logger> loggerHighSpeed;

        std::chrono::milliseconds samplePeriodLowSpeed;
        std::chrono::milliseconds loggingPeriodLowSpeed;
        std::chrono::milliseconds samplePeriodHighSpeed;

        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeed;
        rclcpp::TimerBase::SharedPtr timerDataLoggingLowSpeedWrite;
        rclcpp::TimerBase::SharedPtr timerDataLoggingHighSpeed;

        std::chrono::milliseconds ignitionSequenceEnd = -1ms;
        std::chrono::milliseconds ignitionSequenceOpenBV = -1ms;
        std::chrono::milliseconds ignitionSequenceCloseBV = -1ms;
        std::chrono::milliseconds ignitionSequenceOpenPV = -1ms;
        std::chrono::milliseconds ignitionSequenceClosePV = -1ms;
        std::chrono::milliseconds ignitionSequenceOpenMV1 = -1ms;
        std::chrono::milliseconds ignitionSequenceCloseMV1 = -1ms;
        std::chrono::milliseconds ignitionSequenceOpenMV2 = -1ms;
        std::chrono::milliseconds ignitionSequenceCloseMV2 = -1ms;
        std::chrono::milliseconds ignitionSequenceOpenPurge = -1ms;
        std::chrono::milliseconds ignitionSequenceClosePurge = -1ms;
        std::chrono::milliseconds ignitionSequenceOnIgnitor = -1ms;
        std::chrono::milliseconds ignitionSequenceOffIgnitor = -1ms;
        std::chrono::milliseconds ignitionSequenceOnHSDatalogging = -1ms;
        std::chrono::milliseconds ignitionSequenceOffHSDatalogging = -1ms;

        bool ignitionSequenceOpenBVTriggered = false;
        bool ignitionSequenceCloseBVTriggered = false;
        bool ignitionSequenceOpenPVTriggered = false;
        bool ignitionSequenceClosePVTriggered = false;
        bool ignitionSequenceOpenMV1Triggered = false;
        bool ignitionSequenceCloseMV1Triggered = false;
        bool ignitionSequenceOpenMV2Triggered = false;
        bool ignitionSequenceCloseMV2Triggered = false;
        bool ignitionSequenceOpenPurgeTriggered = false;
        bool ignitionSequenceClosePurgeTriggered = false;
        bool ignitionSequenceOnIgnitorTriggered = false;
        bool ignitionSequenceOffIgnitorTriggered = false;
        bool ignitionSequenceOnHSDataloggingTriggered = false;
        bool ignitionSequenceOffHSDataloggingTriggered = false;

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

        void readConfig(toml::table toml);

        void callbackDataLoggingLowSpeed();
        void callbackDataLoggingLowSpeedWrite();
        void callbackDataLoggingHighSpeed();
        void ignitionSequenceCallback();

        recu_msgs::srv::GetValveState::Response createValveStateResponse(ValveState);

        void IgnitionSequence(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                              std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void Abort(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                   std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void BV_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                     std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void BV_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void BV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                         std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void PV_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                     std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void PV_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void PV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                         std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void MV1_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void MV1_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void MV1_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void MV2_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void MV2_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void MV2_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);
        void Purge_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                      std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void Purge_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request>,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response>);
        void Purge_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request>,
                          std::shared_ptr<recu_msgs::srv::GetValveState::Response>);

        void deleteAllPointers();
    };
} // namespace recu

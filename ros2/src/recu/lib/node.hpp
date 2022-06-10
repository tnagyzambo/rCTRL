#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <recu_msgs/msg/valve_state.hpp>
#include <recu_msgs/srv/get_valve_state.hpp>
#include <recu_msgs/srv/valve_action.hpp>
#include <rgpio/input.hpp>
#include <rgpio/output.hpp>
#include <rutil/fmt.hpp>
#include <rutil/toml.hpp>
#include <string>

#define GPIO_CONFIG_FILE "/home/ros/rocketGPIO/config.toml"

namespace recu {
    enum ValveState {
        Closed,
        Open,
    };

    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

    private:
        std::unique_ptr<rgpio::Output> valveBV;
        std::unique_ptr<rgpio::Output> valvePV;
        std::unique_ptr<rgpio::Output> valveMV1;
        std::unique_ptr<rgpio::Output> valveMV2;
        std::unique_ptr<rgpio::Output> pyro;

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

        recu_msgs::srv::GetValveState::Response createValveStateResponse(ValveState);

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

        void deleteAllPointers();
    };
} // namespace recu

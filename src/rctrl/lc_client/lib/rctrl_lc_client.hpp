#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

using namespace std::chrono_literals;

namespace rctrl::lc
{
    class Client
    {
    public:
        Client(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
               const char *serviceNodeName,
               std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>,
               std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>);
        ~Client();

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr callingNode;
        const char *serviceNodeName;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> clGetState;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> clChangeState;

        unsigned int getState(std::chrono::seconds);
        bool changeState(std::uint8_t, std::chrono::seconds);

    private:
        template <typename FutureT, typename WaitTimeT>
        static std::future_status waitForResult(FutureT &, WaitTimeT);
    };

// Implementation of templated functions
#include <rctrl_lc_client.tpp>
}

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <rctrl_lc_client.hpp>
#include <rdata_iface.hpp>

#include <rcl_interfaces/msg/log.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

using namespace std::chrono_literals;

namespace rdata::lc_client
{
    class Node : public rclcpp::Node
    {
    public:
        Node(const char *nodeName);
        ~Node();

    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr timer2;

        std::unique_ptr<rctrl::lc::Client> rdata;
        std::unique_ptr<rctrl::lc::Client> vF64;

        void initNcurses();
        lifecycle_msgs::msg::Transition

        rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subRosout;
        void callbackSubRosout(const rcl_interfaces::msg::Log::SharedPtr) const;

        void run();
    };
}
/**
     * This is a little independent
     * script which triggers the
     * default lifecycle of a node.
     * It starts with configure, activate,
     * deactivate, activate, deactivate,
     * cleanup and finally shutdown
     */
// void callee_script(std::shared_ptr<Node> lc_client)
// {
//     rclcpp::WallRate time_between_state_changes(0.1); // 10s

//     // configure
//     {
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // activate
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // deactivate
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // activate it again
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // and deactivate it again
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // we cleanup
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }

//     // and finally shutdown
//     // Note: We have to be precise here on which shutdown transition id to call
//     // We are currently in the unconfigured state and thus have to call
//     // TRANSITION_UNCONFIGURED_SHUTDOWN
//     {
//         time_between_state_changes.sleep();
//         if (!rclcpp::ok())
//         {
//             return;
//         }
//         if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
//         {
//             return;
//         }
//         if (!lc_client->get_state())
//         {
//             return;
//         }
//     }
// }

#pragma once

#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sol/sol.hpp>

#include <rstate/action/transition.hpp>

#include <rstate_state.hpp>

namespace rstate
{
    // Forward declaration to resolve circular dependency/include
    class State;

    class Node : public rclcpp::Node
    {
    public:
        explicit Node();

        rclcpp_action::Server<rstate::action::Transition>::SharedPtr actionServer;

        void setState(State &);

    private:
        State *currentState;
    };

}

// RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::ActionServer)

#include "rstate_state.hpp"
#include <rstate_node.hpp>

rstate::Node::Node() : rclcpp::Node("rstate")
{
    this->setState(Unconfigured::getInstance());

    this->actionServer = rclcpp_action::create_server<rstate::action::Transition>(
        this,
        "rstate/transition",
        std::bind(&State::handleGoal, std::ref(this->currentState), this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&State::handleCancel, std::ref(this->currentState), this, std::placeholders::_1),
        std::bind(&State::handleAccepted, std::ref(this->currentState), this, std::placeholders::_1));
}

void rstate::Node::setState(State &newState)
{
    // this->currentState->exit();     // do stuff before we change state
    this->currentState = &newState;  // change state
    this->currentState->enter(this); // do stuff after we change state
}

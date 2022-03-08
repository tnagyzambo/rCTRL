#include "rstate_state.hpp"
#include <rstate_node.hpp>

namespace rstate {
    Node::Node() : rclcpp::Node("rstate") {
        this->setState(Unconfigured::getInstance());

        this->actionServer = rclcpp_action::create_server<action::Transition>(
            this,
            "rstate/transition",
            std::bind(&State::handleGoal, std::ref(this->currentState), this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&State::handleCancel, std::ref(this->currentState), this, std::placeholders::_1),
            std::bind(&State::handleAccepted, std::ref(this->currentState), this, std::placeholders::_1));
    }

    void Node::setState(State &newState) {
        this->currentState = &newState;
        this->currentState->enter(this);
    }
} // namespace rstate

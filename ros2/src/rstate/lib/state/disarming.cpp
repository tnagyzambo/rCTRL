#include <state/state.hpp>

namespace rstate {
    State &Disarming::getInstance() {
        static Disarming singleton;
        return singleton;
    }

    void Disarming::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is disarming"); }

    rstate::msg::NetworkState Disarming::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Disarming;
        network_state.label = "disarming";

        return network_state;
    }

    GoalResponse Disarming::handleGoal(Node *node,
                                       std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while disarming, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Disarming::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Disarming::handleAccepted(Node *node,
                                   const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

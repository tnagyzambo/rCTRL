#include <state/state.hpp>

namespace rstate {
    State &ShuttingDown::getInstance() {
        static ShuttingDown singleton;
        return singleton;
    }

    void ShuttingDown::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is shutting down"); }

    rstate::msg::NetworkState ShuttingDown::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::ShuttingDown;
        network_state.label = "shutting_down";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response ShuttingDown::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse ShuttingDown::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while shutting down, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse ShuttingDown::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void ShuttingDown::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

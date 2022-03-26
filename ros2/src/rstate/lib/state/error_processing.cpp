#include <state/state.hpp>

namespace rstate {
    State &ErrorProcessing::getInstance() {
        static ErrorProcessing singleton;
        return singleton;
    }

    void ErrorProcessing::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is error processing"); }

    rstate::msg::NetworkState ErrorProcessing::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::ErrorProcessing;
        network_state.label = "error_processing";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response ErrorProcessing::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse ErrorProcessing::handleGoal(Node *node,
                                             std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while error processing, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse ErrorProcessing::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void ErrorProcessing::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

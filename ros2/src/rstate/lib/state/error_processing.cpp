#include <state/state.hpp>

namespace rstate {
    State &ErrorProcessing::getInstance() {
        static ErrorProcessing singleton;
        return singleton;
    }

    void ErrorProcessing::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is error processing"); }

    rstate_msgs::msg::NetworkState ErrorProcessing::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::ErrorProcessing;
        network_state.label = "error_processing";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response ErrorProcessing::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse ErrorProcessing::handleGoal(
        Node *node, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while error processing, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse ErrorProcessing::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void ErrorProcessing::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

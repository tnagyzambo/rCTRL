#include <state/state.hpp>

namespace rstate {
    State &Deactivating::getInstance() {
        static Deactivating singleton;
        return singleton;
    }

    void Deactivating::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is deactivating"); }

    rstate::msg::NetworkState Deactivating::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Deactivating;
        network_state.label = "deactivating";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response Deactivating::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Deactivating::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while deactivating, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Deactivating::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Deactivating::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

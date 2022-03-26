#include <state/state.hpp>

namespace rstate {
    State &Arming::getInstance() {
        static Arming singleton;
        return singleton;
    }

    void Arming::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is arming"); }

    rstate::msg::NetworkState Arming::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Arming;
        network_state.label = "arming";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response Arming::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Arming::handleGoal(Node *node,
                                    std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while arming, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Arming::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Arming::handleAccepted(Node *node,
                                const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

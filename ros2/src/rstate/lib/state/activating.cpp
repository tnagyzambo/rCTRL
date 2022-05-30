#include <state/state.hpp>

namespace rstate {
    State &Activating::getInstance() {
        static Activating singleton;
        return singleton;
    }

    void Activating::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is activating"); }

    rstate_msgs::msg::NetworkState Activating::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Activating;
        network_state.label = "activating";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Activating::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Activating::handleGoal(Node *node,
                                        std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while activating, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Activating::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Activating::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

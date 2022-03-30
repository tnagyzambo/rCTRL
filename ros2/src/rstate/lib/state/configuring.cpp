#include <state/state.hpp>

namespace rstate {
    State &Configuring::getInstance() {
        static Configuring singleton;
        return singleton;
    }

    void Configuring::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is configuring"); }

    rstate_msgs::msg::NetworkState Configuring::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Configuring;
        network_state.label = "configuring";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Configuring::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Configuring::handleGoal(Node *node,
                                         std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while configuring, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Configuring::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Configuring::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

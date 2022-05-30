#include <state/state.hpp>

namespace rstate {
    State &CleaningUp::getInstance() {
        static CleaningUp singleton;
        return singleton;
    }

    void CleaningUp::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is cleaning up"); }

    rstate_msgs::msg::NetworkState CleaningUp::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::CleaningUp;
        network_state.label = "cleaning_up";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response CleaningUp::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse CleaningUp::handleGoal(Node *node,
                                        std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while cleaning up, goal: %d", goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse CleaningUp::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void CleaningUp::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

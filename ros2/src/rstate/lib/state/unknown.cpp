#include <state/state.hpp>

namespace rstate {
    State &Unknown::getInstance() {
        static Unknown singleton;
        return singleton;
    }

    void Unknown::enter(Node *node) { (void)node; }

    rstate_msgs::msg::NetworkState Unknown::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Unknown;
        network_state.label = "unkown";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Unknown::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Unknown::handleGoal(Node *node,
                                     std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(),
                    "Received action request while in an unknown network state, goal: %d",
                    goal->transition.id);
        return GoalResponse::REJECT;
    }

    CancelResponse Unknown::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
        return CancelResponse::REJECT;
    }

    void Unknown::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

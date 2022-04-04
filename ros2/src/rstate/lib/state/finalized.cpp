#include <state/state.hpp>

namespace rstate {
    // Finalized
    State &Finalized::getInstance() {
        static Finalized singleton;
        return singleton;
    }

    void Finalized::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is finalized"); }

    rstate_msgs::msg::NetworkState Finalized::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Finalized;
        network_state.label = "finalized";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Finalized::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        return response;
    }

    GoalResponse Finalized::handleGoal(Node *node,
                                       std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        (void)node;
        (void)goal;
        return GoalResponse::REJECT;
    }

    CancelResponse Finalized::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
        return CancelResponse::REJECT;
    }

    void Finalized::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

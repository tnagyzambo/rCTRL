#include <state/state.hpp>

namespace rstate {
    // Finalized
    State &Finalized::getInstance() {
        static Finalized singleton;
        return singleton;
    }

    void Finalized::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is finalized"); }

    rstate::msg::NetworkState Finalized::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Finalized;
        network_state.label = "finalized";

        return network_state;
    }

    GoalResponse Finalized::handleGoal(Node *node,
                                       std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        (void)node;
        (void)goal;
        return GoalResponse::REJECT;
    }

    CancelResponse Finalized::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
        return CancelResponse::REJECT;
    }

    void Finalized::handleAccepted(Node *node,
                                   const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

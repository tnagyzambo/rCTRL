#include <state/state.hpp>

namespace rstate {
    State &Arming::getInstance() {
        static Arming singleton;
        return singleton;
    }

    void Arming::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is arming"); }

    GoalResponse Arming::handleGoal(Node *node, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while arming, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse Arming::handleCancel(Node *node,
                                        const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Arming::handleAccepted(Node *node,
                                const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

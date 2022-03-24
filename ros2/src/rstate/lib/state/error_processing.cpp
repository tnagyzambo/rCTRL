#include <state/state.hpp>

namespace rstate {
    State &ErrorProcessing::getInstance() {
        static ErrorProcessing singleton;
        return singleton;
    }

    void ErrorProcessing::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is error processing"); }

    GoalResponse ErrorProcessing::handleGoal(Node *node,
                                             std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while error processing, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse ErrorProcessing::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void ErrorProcessing::handleAccepted(Node *node,
                                         const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

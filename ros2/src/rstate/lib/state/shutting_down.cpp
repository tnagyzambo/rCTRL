#include <state/state.hpp>

namespace rstate {
    State &ShuttingDown::getInstance() {
        static ShuttingDown singleton;
        return singleton;
    }

    void ShuttingDown::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is shutting down"); }

    GoalResponse ShuttingDown::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while shutting down, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse ShuttingDown::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void ShuttingDown::handleAccepted(Node *node,
                                      const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

#include <state/state.hpp>

namespace rstate {
    State &ShuttingDown::getInstance() {
        static ShuttingDown singleton;
        return singleton;
    }

    void ShuttingDown::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is shutting down"); }

    rclcpp_action::GoalResponse ShuttingDown::handleGoal(Node *node,
                                                         const rclcpp_action::GoalUUID &uuid,
                                                         std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while shutting down, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse ShuttingDown::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ShuttingDown::handleAccepted(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

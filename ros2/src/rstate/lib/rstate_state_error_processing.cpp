#include <rstate_state.hpp>

namespace rstate {
    State &ErrorProcessing::getInstance() {
        static ErrorProcessing singleton;
        return singleton;
    }

    void ErrorProcessing::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is error processing"); }

    rclcpp_action::GoalResponse ErrorProcessing::handleGoal(Node *node,
                                                            const rclcpp_action::GoalUUID &uuid,
                                                            std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while error processing, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse ErrorProcessing::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ErrorProcessing::handleAccepted(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

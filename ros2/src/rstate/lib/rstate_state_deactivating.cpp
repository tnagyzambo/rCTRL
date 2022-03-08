#include <rstate_state.hpp>

namespace rstate {
    State &Deactivating::getInstance() {
        static Deactivating singleton;
        return singleton;
    }

    void Deactivating::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is deactivating"); }

    rclcpp_action::GoalResponse Deactivating::handleGoal(Node *node,
                                                         const rclcpp_action::GoalUUID &uuid,
                                                         std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while deactivating, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse Deactivating::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Deactivating::handleAccepted(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

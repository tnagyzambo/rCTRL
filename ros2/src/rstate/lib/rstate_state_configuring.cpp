#include <rstate_state.hpp>

namespace rstate {
    State &Configuring::getInstance() {
        static Configuring singleton;
        return singleton;
    }

    void Configuring::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is configuring"); }

    rclcpp_action::GoalResponse Configuring::handleGoal(Node *node,
                                                        const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while configuring, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse Configuring::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Configuring::handleAccepted(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

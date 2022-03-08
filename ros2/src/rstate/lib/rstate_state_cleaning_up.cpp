#include <rstate_state.hpp>

namespace rstate {
    State &CleaningUp::getInstance() {
        static CleaningUp singleton;
        return singleton;
    }

    void CleaningUp::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is cleaning up"); }

    rclcpp_action::GoalResponse CleaningUp::handleGoal(Node *node,
                                                       const rclcpp_action::GoalUUID &uuid,
                                                       std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while cleaning up, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse CleaningUp::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CleaningUp::handleAccepted(Node *node,
                                    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

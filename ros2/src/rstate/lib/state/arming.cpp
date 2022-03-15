#include <state/state.hpp>

namespace rstate {
    State &Arming::getInstance() {
        static Arming singleton;
        return singleton;
    }

    void Arming::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is arming"); }

    rclcpp_action::GoalResponse Arming::handleGoal(Node *node,
                                                   const rclcpp_action::GoalUUID &uuid,
                                                   std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while arming, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse Arming::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Arming::handleAccepted(Node *node,
                                const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

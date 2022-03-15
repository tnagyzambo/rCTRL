#include <state/state.hpp>

namespace rstate {
    State &Disarming::getInstance() {
        static Disarming singleton;
        return singleton;
    }

    void Disarming::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is disarming"); }

    rclcpp_action::GoalResponse Disarming::handleGoal(Node *node,
                                                      const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received action request while disarming, goal: %d", goal->transition);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse Disarming::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Disarming::handleAccepted(Node *node,
                                   const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

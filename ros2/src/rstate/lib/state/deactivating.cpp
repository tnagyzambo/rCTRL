#include <state/state.hpp>

namespace rstate {
    State &Deactivating::getInstance() {
        static Deactivating singleton;
        return singleton;
    }

    void Deactivating::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is deactivating"); }

    GoalResponse Deactivating::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while deactivating, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse Deactivating::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Deactivating::handleAccepted(Node *node,
                                      const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

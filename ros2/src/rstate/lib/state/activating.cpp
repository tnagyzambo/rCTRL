#include <state/state.hpp>

namespace rstate {
    State &Activating::getInstance() {
        static Activating singleton;
        return singleton;
    }

    void Activating::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is activating"); }

    GoalResponse Activating::handleGoal(Node *node, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while activating, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse Activating::handleCancel(Node *node,
                                            const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Activating::handleAccepted(Node *node,
                                    const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

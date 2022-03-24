#include <state/state.hpp>

namespace rstate {
    State &CleaningUp::getInstance() {
        static CleaningUp singleton;
        return singleton;
    }

    void CleaningUp::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is cleaning up"); }

    GoalResponse CleaningUp::handleGoal(Node *node, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        RCLCPP_INFO(node->get_logger(), "Received action request while cleaning up, goal: %d", goal->transition);
        return GoalResponse::REJECT;
    }

    CancelResponse CleaningUp::handleCancel(Node *node,
                                            const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void CleaningUp::handleAccepted(Node *node,
                                    const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

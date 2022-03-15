#include <state/state.hpp>

namespace rstate {
    // Finalized
    State &Finalized::getInstance() {
        static Finalized singleton;
        return singleton;
    }

    void Finalized::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is finalized"); }

    rclcpp_action::GoalResponse Finalized::handleGoal(Node *node,
                                                      const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const action::Transition::Goal> goal) {
        (void)node;
        (void)goal;
        (void)uuid;
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse Finalized::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
        return rclcpp_action::CancelResponse::REJECT;
    }

    void Finalized::handleAccepted(Node *node,
                                   const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        (void)node;
        (void)goalHandle;
    }
} // namespace rstate

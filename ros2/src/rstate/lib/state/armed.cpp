#include <state/state.hpp>

namespace rstate {
    State &Armed::getInstance() {
        static Armed singleton;
        return singleton;
    }

    void Armed::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is armed"); }

    rclcpp_action::GoalResponse Armed::handleGoal(Node *node,
                                                  const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        switch (goal->transition) {
        case (int)Transition::Disarm:
            this->onTransition = &Armed::onDisarm;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Armed::onShutdown;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse Armed::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Armed::handleAccepted(Node *node,
                               const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        std::thread{std::bind(Armed::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Armed::onDisarm(Node *node,
                         const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(Disarming::getInstance());

        switch (executeCommands(node, node->cmdsOnDisarm, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Active::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Armed::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Armed::onShutdown(Node *node,
                           const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownArmed, goalHandle)) {
        case ShutdownResult::Success:
            break;
        case ShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate
#include <state/state.hpp>

namespace rstate {
    State &Active::getInstance() {
        static Active singleton;
        return singleton;
    }

    void Active::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is active"); }

    rclcpp_action::GoalResponse Active::handleGoal(Node *node,
                                                   const rclcpp_action::GoalUUID &uuid,
                                                   std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        switch (goal->transition) {
        case (int)Transition::Arm:
            this->onTransition = &Active::onArm;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Deactivate:
            this->onTransition = &Active::onDeactivate;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Active::onShutdown;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse Active::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Active::handleAccepted(Node *node,
                                const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        std::thread{std::bind(Active::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Active::onArm(Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(Arming::getInstance());

        switch (executeCommands(node, node->cmdsOnArm, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Armed::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Active::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onDeactivate(Node *node,
                              const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(Deactivating::getInstance());

        switch (executeCommands(node, node->cmdsOnDeactivate, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Inactive::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Active::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onShutdown(Node *node,
                            const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownActive, goalHandle)) {
        case ShutdownResult::Success:
            break;
        case ShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

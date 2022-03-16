#include <state/state.hpp>

namespace rstate {
    State &Inactive::getInstance() {
        static Inactive singleton;
        return singleton;
    }

    void Inactive::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is inactive"); }

    rclcpp_action::GoalResponse Inactive::handleGoal(Node *node,
                                                     const rclcpp_action::GoalUUID &uuid,
                                                     std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        switch (goal->transition) {
        case (int)Transition::CleanUp:
            this->onTransition = &Inactive::onCleanUp;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Activate:
            this->onTransition = &Inactive::onActivate;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Inactive::onShutdown;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse Inactive::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Inactive::handleAccepted(Node *node,
                                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        std::thread{std::bind(Inactive::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Inactive::onCleanUp(Node *node,
                             const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(CleaningUp::getInstance());

        switch (executeCommands(node, node->cmdsOnCleanUp, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Unconfigured::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onActivate(Node *node,
                              const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(Activating::getInstance());

        switch (executeCommands(node, node->cmdsOnActivate, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Active::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onShutdown(Node *node,
                              const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdOnShutdownInactive, goalHandle)) {
        case ShutdownResult::Success:
            break;
        case ShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate
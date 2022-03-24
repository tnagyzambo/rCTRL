#include <state/state.hpp>

namespace rstate {
    State &Active::getInstance() {
        static Active singleton;
        return singleton;
    }

    void Active::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is active"); }

    GoalResponse Active::handleGoal(Node *node, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        switch (goal->transition) {
        case (int)Transition::Arm:
            this->onTransition = &Active::onArm;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Deactivate:
            this->onTransition = &Active::onDeactivate;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Active::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Active::handleCancel(Node *node,
                                        const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Active::handleAccepted(Node *node,
                                const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        std::thread{std::bind(Active::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Active::onArm(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
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

    void Active::onDeactivate(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
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

    void Active::onShutdown(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
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

#include <state/state.hpp>

namespace rstate {
    State &Armed::getInstance() {
        static Armed singleton;
        return singleton;
    }

    void Armed::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is armed"); }

    GoalResponse Armed::handleGoal(Node *node, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request> goal) {
        switch (goal->transition) {
        case (int)Transition::Disarm:
            this->onTransition = &Armed::onDisarm;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Armed::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Armed::handleCancel(Node *node,
                                       const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Armed::handleAccepted(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
        std::thread{std::bind(Armed::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Armed::onDisarm(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
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

    void Armed::onShutdown(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>> goalHandle) {
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

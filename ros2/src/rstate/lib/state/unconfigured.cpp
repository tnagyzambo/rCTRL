#include <state.hpp>

namespace rstate {
    State &Unconfigured::getInstance() {
        static Unconfigured singleton;
        return singleton;
    }

    void Unconfigured::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is unconfigured"); }

    rclcpp_action::GoalResponse Unconfigured::handleGoal(Node *node,
                                                         const rclcpp_action::GoalUUID &uuid,
                                                         std::shared_ptr<const action::Transition::Goal> goal) {
        (void)uuid;
        switch (goal->transition) {
        case (int)Transition::Configure:
            this->onTransition = &Unconfigured::onConfigure;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)Transition::Shutdown:
            this->onTransition = &Unconfigured::onShutdown;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse Unconfigured::handleCancel(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Unconfigured::handleAccepted(
        Node *node, const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        std::thread{
            std::bind(Unconfigured::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Unconfigured::onConfigure(Node *node,
                                   const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(Configuring::getInstance());

        switch (executeCommands(node, node->cmdsOnConfigure, goalHandle)) {
        case TransitionResult::Success:
            node->setState(Inactive::getInstance());
            break;
        case TransitionResult::Cancelled:
            node->setState(Unconfigured::getInstance());
            break;
        case TransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Unconfigured::onShutdown(Node *node,
                                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownUnconfigured, goalHandle)) {
        case ShutdownResult::Success:
            break;
        case ShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

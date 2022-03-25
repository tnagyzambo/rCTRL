#include <state/state.hpp>

namespace rstate {
    State &Inactive::getInstance() {
        static Inactive singleton;
        return singleton;
    }

    void Inactive::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is inactive"); }

    rstate::msg::NetworkState Inactive::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Inactive;
        network_state.label = "inactive";

        return network_state;
    }

    GoalResponse Inactive::handleGoal(Node *node,
                                      std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransition::CleanUp:
            this->onTransition = &Inactive::onCleanUp;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Activate:
            this->onTransition = &Inactive::onActivate;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Shutdown:
            this->onTransition = &Inactive::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Inactive::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Inactive::handleAccepted(Node *node,
                                  const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{std::bind(Inactive::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Inactive::onCleanUp(Node *node,
                             const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(CleaningUp::getInstance());

        switch (executeCommands(node, node->cmdsOnCleanUp, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Unconfigured::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onActivate(Node *node,
                              const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Activating::getInstance());

        switch (executeCommands(node, node->cmdsOnActivate, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onShutdown(Node *node,
                              const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdOnShutdownInactive, goalHandle)) {
        case NetworkShutdownResult::Success:
            break;
        case NetworkShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

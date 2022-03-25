#include <state/state.hpp>

namespace rstate {
    State &Active::getInstance() {
        static Active singleton;
        return singleton;
    }

    void Active::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is active"); }

    rstate::msg::NetworkState Active::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Active;
        network_state.label = "active";

        return network_state;
    }

    GoalResponse Active::handleGoal(Node *node,
                                    std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransition::Arm:
            this->onTransition = &Active::onArm;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Deactivate:
            this->onTransition = &Active::onDeactivate;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Shutdown:
            this->onTransition = &Active::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Active::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Active::handleAccepted(Node *node,
                                const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{std::bind(Active::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Active::onArm(Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Arming::getInstance());

        switch (executeCommands(node, node->cmdsOnArm, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Armed::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onDeactivate(Node *node,
                              const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Deactivating::getInstance());

        switch (executeCommands(node, node->cmdsOnDeactivate, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onShutdown(Node *node,
                            const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownActive, goalHandle)) {
        case NetworkShutdownResult::Success:
            break;
        case NetworkShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

#include <state/state.hpp>

namespace rstate {
    State &Armed::getInstance() {
        static Armed singleton;
        return singleton;
    }

    void Armed::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is armed"); }

    rstate::msg::NetworkState Armed::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Armed;
        network_state.label = "armed";

        return network_state;
    }

    GoalResponse Armed::handleGoal(Node *node,
                                   std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransition::Disarm:
            this->onTransition = &Armed::onDisarm;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Shutdown:
            this->onTransition = &Armed::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Armed::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Armed::handleAccepted(Node *node,
                               const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{std::bind(Armed::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Armed::onDisarm(Node *node,
                         const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Disarming::getInstance());

        switch (executeCommands(node, node->cmdsOnDisarm, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Armed::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Armed::onShutdown(Node *node,
                           const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownArmed, goalHandle)) {
        case NetworkShutdownResult::Success:
            break;
        case NetworkShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

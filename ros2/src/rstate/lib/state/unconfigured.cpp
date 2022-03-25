#include <state.hpp>

namespace rstate {
    State &Unconfigured::getInstance() {
        static Unconfigured singleton;
        return singleton;
    }

    void Unconfigured::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is unconfigured"); }

    rstate::msg::NetworkState Unconfigured::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkState::Unconfigured;
        network_state.label = "unconfigured";

        return network_state;
    }

    GoalResponse Unconfigured::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransition::Configure:
            this->onTransition = &Unconfigured::onConfigure;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransition::Shutdown:
            this->onTransition = &Unconfigured::onShutdown;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Unconfigured::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Unconfigured::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{
            std::bind(Unconfigured::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Unconfigured::onConfigure(Node *node,
                                   const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Configuring::getInstance());

        switch (executeCommands(node, node->cmdsOnConfigure, goalHandle)) {
        case NetworkTransitionResult::Success:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResult::Cancelled:
            node->setState(Unconfigured::getInstance());
            break;
        case NetworkTransitionResult::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Unconfigured::onShutdown(Node *node,
                                  const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownUnconfigured, goalHandle)) {
        case NetworkShutdownResult::Success:
            break;
        case NetworkShutdownResult::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

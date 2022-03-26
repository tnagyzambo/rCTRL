#include <state.hpp>

namespace rstate {
    State &Unconfigured::getInstance() {
        static Unconfigured singleton;
        return singleton;
    }

    void Unconfigured::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is unconfigured"); }

    rstate::msg::NetworkState Unconfigured::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Unconfigured;
        network_state.label = "unconfigured";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response Unconfigured::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        response.available_transitions = {
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Configure, NetworkStateEnum::Unconfigured, NetworkStateEnum::Inactive),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Unconfigured, NetworkStateEnum::Finalized)};
        return response;
    }

    GoalResponse Unconfigured::handleGoal(Node *node,
                                          std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransitionEnum::Configure:
            this->onTransition = &Unconfigured::onConfigure;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Configure, NetworkStateEnum::Unconfigured, NetworkStateEnum::Inactive);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Shutdown:
            this->onTransition = &Unconfigured::onShutdown;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Unconfigured, NetworkStateEnum::Finalized);
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
        case NetworkTransitionResultEnum::Success:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Unconfigured::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Unconfigured::onShutdown(Node *node,
                                  const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownUnconfigured, goalHandle)) {
        case NetworkShutdownResultEnum::Success:
            break;
        case NetworkShutdownResultEnum::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

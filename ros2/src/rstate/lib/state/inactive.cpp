#include <state/state.hpp>

namespace rstate {
    State &Inactive::getInstance() {
        static Inactive singleton;
        return singleton;
    }

    void Inactive::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is inactive"); }

    rstate_msgs::msg::NetworkState Inactive::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Inactive;
        network_state.label = "inactive";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Inactive::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        response.available_transitions = {
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::CleanUp, NetworkStateEnum::Inactive, NetworkStateEnum::Unconfigured),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Activate, NetworkStateEnum::Inactive, NetworkStateEnum::Active),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Inactive, NetworkStateEnum::Finalized)};
        return response;
    }

    GoalResponse Inactive::handleGoal(Node *node,
                                      std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransitionEnum::CleanUp:
            this->onTransition = &Inactive::onCleanUp;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::CleanUp, NetworkStateEnum::Inactive, NetworkStateEnum::Unconfigured);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Activate:
            this->onTransition = &Inactive::onActivate;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Activate, NetworkStateEnum::Inactive, NetworkStateEnum::Active);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Shutdown:
            this->onTransition = &Inactive::onShutdown;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Inactive, NetworkStateEnum::Finalized);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Inactive::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Inactive::handleAccepted(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{std::bind(Inactive::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Inactive::onCleanUp(Node *node,
                             const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(CleaningUp::getInstance());

        switch (executeCommands(node, node->cmdsOnCleanUp, goalHandle)) {
        case NetworkTransitionResultEnum::Success:
            node->setState(Unconfigured::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onActivate(Node *node,
                              const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Activating::getInstance());

        switch (executeCommands(node, node->cmdsOnActivate, goalHandle)) {
        case NetworkTransitionResultEnum::Success:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Inactive::onShutdown(Node *node,
                              const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdOnShutdownInactive, goalHandle)) {
        case NetworkShutdownResultEnum::Success:
            break;
        case NetworkShutdownResultEnum::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

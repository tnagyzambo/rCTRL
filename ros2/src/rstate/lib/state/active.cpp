#include <algorithm>
#include <state/state.hpp>

namespace rstate {
    State &Active::getInstance() {
        static Active singleton;
        return singleton;
    }

    void Active::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is active"); }

    rstate::msg::NetworkState Active::getNetworkState() {
        rstate::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Active;
        network_state.label = "active";

        return network_state;
    }

    rstate::srv::GetAvailableNetworkTransitions::Response Active::getAvailableNetworkTransitions() {
        rstate::srv::GetAvailableNetworkTransitions::Response response;
        response.available_transitions = {
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Arm, NetworkStateEnum::Active, NetworkStateEnum::Armed),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Deactivate, NetworkStateEnum::Active, NetworkStateEnum::Inactive),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Active, NetworkStateEnum::Finalized)};
        return response;
    }

    GoalResponse Active::handleGoal(Node *node,
                                    std::shared_ptr<const rstate::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransitionEnum::Arm:
            this->onTransition = &Active::onArm;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Arm, NetworkStateEnum::Active, NetworkStateEnum::Armed);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Deactivate:
            this->onTransition = &Active::onDeactivate;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Deactivate, NetworkStateEnum::Active, NetworkStateEnum::Inactive);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Shutdown:
            this->onTransition = &Active::onShutdown;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Active, NetworkStateEnum::Finalized);
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
        case NetworkTransitionResultEnum::Success:
            node->setState(Armed::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onDeactivate(Node *node,
                              const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Deactivating::getInstance());

        switch (executeCommands(node, node->cmdsOnDeactivate, goalHandle)) {
        case NetworkTransitionResultEnum::Success:
            node->setState(Inactive::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Active::onShutdown(Node *node,
                            const std::shared_ptr<GoalHandle<rstate::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownActive, goalHandle)) {
        case NetworkShutdownResultEnum::Success:
            break;
        case NetworkShutdownResultEnum::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

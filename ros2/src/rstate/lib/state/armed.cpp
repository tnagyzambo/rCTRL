#include <state/state.hpp>

namespace rstate {
    State &Armed::getInstance() {
        static Armed singleton;
        return singleton;
    }

    void Armed::enter(Node *node) { RCLCPP_INFO(node->get_logger(), "Network is armed"); }

    rstate_msgs::msg::NetworkState Armed::getNetworkState() {
        rstate_msgs::msg::NetworkState network_state;
        network_state.id = (uint)NetworkStateEnum::Armed;
        network_state.label = "armed";

        return network_state;
    }

    rstate_msgs::srv::GetAvailableNetworkTransitions::Response Armed::getAvailableNetworkTransitions() {
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response response;
        response.available_transitions = {
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Disarm, NetworkStateEnum::Armed, NetworkStateEnum::Active),
            generateNetworkTransitionDescription(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Armed, NetworkStateEnum::Finalized)};
        return response;
    }

    GoalResponse Armed::handleGoal(Node *node,
                                   std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request> goal) {
        switch (goal->transition.id) {
        case (int)NetworkTransitionEnum::Disarm:
            this->onTransition = &Armed::onDisarm;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Disarm, NetworkStateEnum::Armed, NetworkStateEnum::Active);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        case (int)NetworkTransitionEnum::Shutdown:
            this->onTransition = &Armed::onShutdown;
            node->publishNetworkTransitionEvent(
                NetworkTransitionEnum::Shutdown, NetworkStateEnum::Armed, NetworkStateEnum::Finalized);
            return GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition.id);
            return GoalResponse::REJECT;
        }
    }

    CancelResponse Armed::handleCancel(
        Node *node, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return CancelResponse::ACCEPT;
    }

    void Armed::handleAccepted(Node *node,
                               const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        std::thread{std::bind(Armed::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}
            .detach();
    }

    void Armed::onDisarm(Node *node,
                         const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(Disarming::getInstance());

        switch (executeCommands(node, node->cmdsOnDisarm, goalHandle)) {
        case NetworkTransitionResultEnum::Success:
            node->setState(Active::getInstance());
            break;
        case NetworkTransitionResultEnum::Cancelled:
            node->setState(Armed::getInstance());
            break;
        case NetworkTransitionResultEnum::Failure:
            // SHUTDOWN
            break;
        }
    }

    void Armed::onShutdown(Node *node,
                           const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        switch (executeCommandsShutdown(node, node->cmdsOnShutdownArmed, goalHandle)) {
        case NetworkShutdownResultEnum::Success:
            break;
        case NetworkShutdownResultEnum::Failure:
            RCLCPP_ERROR(node->get_logger(), "The network was not shutdown successfully");
            break;
        }

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

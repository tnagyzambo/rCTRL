#include <rstate_state.hpp>

namespace rstate
{
    State &Active::getInstance()
    {
        static Active singleton;
        return singleton;
    }

    void Active::enter(Node *node)
    {
        RCLCPP_INFO(node->get_logger(), "Network is active");
    }

    rclcpp_action::GoalResponse Active::handleGoal(
        Node *node,
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const action::Transition::Goal> goal)
    {
        (void)uuid;
        switch (goal->transition)
        {
        case Transition::Deactivate:
            this->onTransition = &Active::onDeactivate;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case Transition::Shutdown:
            this->onTransition = &Active::onShutdown;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        default:
            RCLCPP_INFO(node->get_logger(), "Received transition request with invalid goal: %d", goal->transition);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse Active::handleCancel(
        Node *node,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle)
    {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel transition");
        (void)goalHandle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Active::handleAccepted(
        Node *node,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle)
    {
        std::thread{std::bind(Active::onTransition, this, std::placeholders::_1, std::placeholders::_2), node, goalHandle}.detach();
    }

    void Active::onDeactivate(
        Node *node,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle)
    {
        node->setState(Deactivating::getInstance());

        const auto goal = goalHandle->get_goal();

        node->setState(Inactive::getInstance());
    }

    void Active::onShutdown(
        Node *node,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle)
    {
        node->setState(ShuttingDown::getInstance());

        const auto goal = goalHandle->get_goal();

        node->setState(Finalized::getInstance());
    }
}

#include <rstate_state.hpp>

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
        case Transition::Configure:
            this->onTransition = &Unconfigured::onConfigure;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        case Transition::Shutdown:
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
        // Set transition state
        node->setState(Configuring::getInstance());

        auto feedback = std::make_shared<rstate::action::Transition::Feedback>();
        auto result = std::make_shared<rstate::action::Transition::Result>();
        feedback->cmd_total = node->cmdsOnConfigure.size();
        feedback->cmd_complete = 0;
        goalHandle->publish_feedback(feedback);

        std::vector<std::shared_ptr<CmdIface>> cmdsCompleted;
        for (auto &cmd : node->cmdsOnConfigure) // access by reference to avoid copying
        {
            if (goalHandle->is_canceling()) {
            }

            bool cmdResult = cmd->execute();
            if (cmdResult == true) {
                cmdsCompleted.push_back(cmd);
                feedback->cmd_complete = cmdsCompleted.size();
                goalHandle->publish_feedback(feedback);
            } else {
                node->setState(ErrorProcessing::getInstance());

                // Do stuff

                goalHandle->abort(result);
                return;
            }
        }

        if (rclcpp::ok()) {
            goalHandle->succeed(result);
            node->setState(Inactive::getInstance());
        }
    }

    void Unconfigured::onShutdown(Node *node,
                                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        node->setState(ShuttingDown::getInstance());

        const auto goal = goalHandle->get_goal();

        node->setState(Finalized::getInstance());
    }
} // namespace rstate

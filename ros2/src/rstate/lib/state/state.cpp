#include <state.hpp>

namespace rstate {
    TransitionResult State::executeCommands(
        Node *node,
        std::vector<std::shared_ptr<CmdIface>> cmds,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        auto feedback = std::make_shared<rstate::action::Transition::Feedback>();
        auto result = std::make_shared<rstate::action::Transition::Result>();
        feedback->cmd_total = node->cmdsOnConfigure.size();
        feedback->cmd_complete = 0;
        goalHandle->publish_feedback(feedback);

        std::vector<std::shared_ptr<CmdIface>> cmdsCompleted;
        for (auto &cmd : cmds) {
            if (goalHandle->is_canceling()) {
                try {
                    executeCommandsCancel(cmdsCompleted, goalHandle);
                } catch (except::cmd_service_eror e) {
                    node->setState(ErrorProcessing::getInstance());
                    RCLCPP_ERROR(node->get_logger(), "Failed to cancel transition\nError: %s", e.what());
                    goalHandle->abort(result);
                    return TransitionResult::Failure;
                }

                goalHandle->canceled(result);
                return TransitionResult::Cancelled;
            }

            try {
                cmd->execute();
            } catch (except::cmd_service_eror e) {
                node->setState(ErrorProcessing::getInstance());
                RCLCPP_ERROR(node->get_logger(), "Failed to execute command with id '%d'\nError: %s", cmd->id, e.what());

                try {
                    executeCommandsCancel(cmdsCompleted, goalHandle);
                } catch (except::cmd_service_eror e) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to revert partial transition\nError: %s", e.what());
                    goalHandle->abort(result);
                    return TransitionResult::Failure;
                }

                goalHandle->abort(result);
                return TransitionResult::Cancelled;
            }

            cmdsCompleted.push_back(cmd);
            feedback->cmd_complete = cmdsCompleted.size();
            goalHandle->publish_feedback(feedback);
        }

        goalHandle->succeed(result);
        return TransitionResult::Success;
    }

    void State::executeCommandsCancel(std::vector<std::shared_ptr<CmdIface>> cmds,
                                      const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>> goalHandle) {
        auto feedback = std::make_shared<rstate::action::Transition::Feedback>();
        while (!cmds.empty()) {
            auto cmd = cmds.back();

            try {
                cmd->cancel();
            } catch (except::cmd_service_eror e) {
                throw;
            }

            feedback->cmd_complete -= 1;
            goalHandle->publish_feedback(feedback);
            cmds.pop_back();
        }
    }
} // namespace rstate

#include <state.hpp>
#include <string>

namespace rstate {
    NetworkTransitionResultEnum State::executeCommands(
        Node *node,
        std::vector<std::shared_ptr<CmdIface>> cmds,
        const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        auto feedback = rstate_msgs::msg::NetworkTransitionFeedback();
        // auto result = std::make_shared<rstate_msgs::action::NetworkTransition::Result>();
        feedback.cmd_total = cmds.size();
        feedback.cmd_complete = 0;
        goalHandle->publish_feedback(feedback);

        std::vector<std::shared_ptr<CmdIface>> cmdsCompleted;
        for (auto &cmd : cmds) {
            if (goalHandle->is_canceling()) {
                try {
                    executeCommandsCancel(cmdsCompleted, goalHandle);
                } catch (except::cmd_service_eror &e) {
                    node->setState(ErrorProcessing::getInstance());
                    RCLCPP_ERROR(node->get_logger(), "Failed to cancel transition\nError: %s", e.what());
                    // goalHandle->abort(result);
                    return NetworkTransitionResultEnum::Failure;
                }

                // goalHandle->canceled(result);
                return NetworkTransitionResultEnum::Cancelled;
            }

            try {
                cmd->execute();
            } catch (except::cmd_service_eror &e) {
                node->setState(ErrorProcessing::getInstance());

                RCLCPP_ERROR(
                    node->get_logger(), "Failed to execute command!\nError: %s\nTOML: %s", e.what(), cmd->toml.c_str());

                try {
                    executeCommandsCancel(cmdsCompleted, goalHandle);
                } catch (except::cmd_service_eror &e) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to revert partial transition\nError: %s", e.what());
                    // goalHandle->abort(result);
                    return NetworkTransitionResultEnum::Failure;
                }

                // goalHandle->abort(result);
                return NetworkTransitionResultEnum::Cancelled;
            }

            cmdsCompleted.push_back(cmd);
            feedback.cmd_complete = cmdsCompleted.size();
            goalHandle->publish_feedback(feedback);
        }

        // goalHandle->succeed(result);
        return NetworkTransitionResultEnum::Success;
    }

    void State::executeCommandsCancel(
        std::vector<std::shared_ptr<CmdIface>> cmds,
        const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        (void)goalHandle;
        while (!cmds.empty()) {
            auto cmd = cmds.back();

            if (!cmd->allowCancel) {
                std::stringstream error;

                error << "Attempted to cancel non cancelable command!\n";
                error << "TOML: " << cmd->toml << "\n";

                throw except::cmd_service_eror(error.str());
            }

            try {
                // Trying to call cancel without checking allowCancel will reference null pointers
                cmd->cancel();
            } catch (except::cmd_service_eror &e) {
                throw;
            }

            cmds.pop_back();
        }
    }

    NetworkShutdownResultEnum State::executeCommandsShutdown(
        Node *node,
        std::vector<std::shared_ptr<CmdIface>> cmds,
        const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>> goalHandle) {
        auto feedback = rstate_msgs::msg::NetworkTransitionFeedback();
        // auto result = std::make_shared<rstate_msgs::action::NetworkTransition::Result>();
        feedback.cmd_total = cmds.size();
        feedback.cmd_complete = 0;
        goalHandle->publish_feedback(feedback);
        NetworkShutdownResultEnum transitionResult = NetworkShutdownResultEnum::Success;

        std::vector<std::shared_ptr<CmdIface>> cmdsCompleted;
        for (auto &cmd : cmds) {
            try {
                cmd->execute();
            } catch (except::cmd_service_eror &e) {
                std::stringstream cmdToml;
                cmdToml << cmd->toml;
                RCLCPP_ERROR(node->get_logger(),
                             "Failed to execute command!\nError: %s\nTOML: %s",
                             e.what(),
                             cmdToml.str().c_str());
                transitionResult = NetworkShutdownResultEnum::Failure;
            }

            cmdsCompleted.push_back(cmd);
            feedback.cmd_complete = cmdsCompleted.size();
            goalHandle->publish_feedback(feedback);
        }

        if (transitionResult == NetworkShutdownResultEnum::Success) {
            // goalHandle->succeed(result);
        } else {
            // goalHandle->abort(result);
        }

        return transitionResult;
    }

    std::string generateNetworkStateLabel(NetworkStateEnum state) {
        std::string label;
        switch ((uint)state) {
        case (uint)NetworkStateEnum::Unknown:
            label = "unknown";
            break;
        case (uint)NetworkStateEnum::Unconfigured:
            label = "unconfigured";
            break;
        case (uint)NetworkStateEnum::Inactive:
            label = "inactive";
            break;
        case (uint)NetworkStateEnum::Active:
            label = "active";
            break;
        case (uint)NetworkStateEnum::Armed:
            label = "armed";
            break;
        case (uint)NetworkStateEnum::Finalized:
            label = "finalized";
            break;
        case (uint)NetworkStateEnum::Configuring:
            label = "configuring";
            break;
        case (uint)NetworkStateEnum::CleaningUp:
            label = "cleaning_up";
            break;
        case (uint)NetworkStateEnum::Activating:
            label = "activating";
            break;
        case (uint)NetworkStateEnum::Deactivating:
            label = "deactivating";
            break;
        case (uint)NetworkStateEnum::Arming:
            label = "arming";
            break;
        case (uint)NetworkStateEnum::Disarming:
            label = "disarming";
            break;
        case (uint)NetworkStateEnum::ShuttingDown:
            label = "shutting_down";
            break;
        case (uint)NetworkStateEnum::ErrorProcessing:
            label = "error_processing";
            break;
        };
        return label;
    }

    std::string generateNetworkTransitionLabel(NetworkTransitionEnum transition) {
        std::string label;
        switch ((uint)transition) {
        case (uint)NetworkTransitionEnum::Configure:
            label = "configure";
            break;
        case (uint)NetworkTransitionEnum::CleanUp:
            label = "cleanup";
            break;
        case (uint)NetworkTransitionEnum::Activate:
            label = "activate";
            break;
        case (uint)NetworkTransitionEnum::Deactivate:
            label = "deactivate";
            break;
        case (uint)NetworkTransitionEnum::Arm:
            label = "arm";
            break;
        case (uint)NetworkTransitionEnum::Disarm:
            label = "disarm";
            break;
        case (uint)NetworkTransitionEnum::Shutdown:
            label = "shutdown";
            break;
        };
        return label;
    }

    rstate_msgs::msg::NetworkTransitionDescription generateNetworkTransitionDescription(NetworkTransitionEnum transition,
                                                                                        NetworkStateEnum start_state,
                                                                                        NetworkStateEnum goal_state) {
        rstate_msgs::msg::NetworkTransitionDescription transitionDescription;
        transitionDescription.transition.id = (uint)transition;
        transitionDescription.transition.label = generateNetworkTransitionLabel(transition);
        transitionDescription.start_state.id = (uint)start_state;
        transitionDescription.start_state.label = generateNetworkStateLabel(start_state);
        transitionDescription.goal_state.id = (uint)goal_state;
        transitionDescription.goal_state.label = generateNetworkStateLabel(goal_state);

        return transitionDescription;
    }
} // namespace rstate

#pragma once

#include <action/server.hpp>
#include <cmd/cmd.hpp>
#include <node.hpp>
#include <rstate_msgs/msg/network_state.hpp>
#include <rstate_msgs/msg/network_transition_description.hpp>
#include <rstate_msgs/msg/network_transition_feedback.hpp>
#include <rstate_msgs/srv/get_available_network_transitions.hpp>
#include <rstate_msgs/srv/network_transition_send_goal.hpp>
#include <sstream>
#include <string>

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    class Node;

    enum class NetworkStateEnum {
        // Primary stated
        Unknown = 0,
        Unconfigured = 1,
        Inactive = 2,
        Active = 3,
        Armed = 4,
        Finalized = 5,

        // Transistion states
        Configuring = 10,
        CleaningUp = 11,
        Activating = 12,
        Deactivating = 13,
        Arming = 14,
        Disarming = 15,
        ShuttingDown = 16,
        ErrorProcessing = 17
    };

    enum class NetworkTransitionEnum { Configure, CleanUp, Activate, Deactivate, Arm, Disarm, Shutdown };

    enum class NetworkTransitionResultEnum { Success, Cancelled, Failure };

    enum class NetworkShutdownResultEnum { Success, Failure };

    std::string generateNetworkStateLabel(NetworkStateEnum);
    std::string generateNetworkTransitionLabel(NetworkTransitionEnum);
    rstate_msgs::msg::NetworkTransitionDescription generateNetworkTransitionDescription(NetworkTransitionEnum,
                                                                                        NetworkStateEnum,
                                                                                        NetworkStateEnum);

    class State {
    public:
        virtual ~State() {}

        virtual void enter(Node *) = 0;

        virtual rstate_msgs::msg::NetworkState getNetworkState() = 0;
        virtual rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions() = 0;
        virtual GoalResponse handleGoal(Node *,
                                        std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>) = 0;
        virtual CancelResponse handleCancel(
            Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>) = 0;
        virtual void handleAccepted(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>) = 0;

        static NetworkTransitionResultEnum executeCommands(
            Node *,
            std::vector<std::shared_ptr<CmdIface>>,
            const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        static void executeCommandsCancel(std::vector<std::shared_ptr<CmdIface>>,
                                          const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        static NetworkShutdownResultEnum executeCommandsShutdown(
            Node *,
            std::vector<std::shared_ptr<CmdIface>>,
            const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
    };

    class Activating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Activating() {}
        Activating(const Activating &other);
        Activating &operator=(const Activating &other);
    };

    class Active : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Active::*onTransition)(Node *,
                                     const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onArm(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onDeactivate(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Active() {}
        Active(const Active &other);
        Active &operator=(const Active &other);
    };

    class Armed : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Armed::*onTransition)(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onDisarm(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Armed() {}
        Armed(const Armed &other);
        Armed &operator=(const Armed &other);
    };

    class Arming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Arming() {}
        Arming(const Arming &other);
        Arming &operator=(const Arming &other);
    };

    class CleaningUp : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        CleaningUp() {}
        CleaningUp(const CleaningUp &other);
        CleaningUp &operator=(const CleaningUp &other);
    };

    class Configuring : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Configuring() {}
        Configuring(const Configuring &other);
        Configuring &operator=(const Configuring &other);
    };

    class Deactivating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Deactivating() {}
        Deactivating(const Deactivating &other);
        Deactivating &operator=(const Deactivating &other);
    };

    class Disarming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Disarming() {}
        Disarming(const Disarming &other);
        Disarming &operator=(const Disarming &other);
    };

    class ErrorProcessing : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        ErrorProcessing() {}
        ErrorProcessing(const ErrorProcessing &other);
        ErrorProcessing &operator=(const ErrorProcessing &other);
    };

    class Finalized : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Finalized() {}
        Finalized(const Finalized &other);
        Finalized &operator=(const Finalized &other);
    };

    class Inactive : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Inactive::*onTransition)(Node *,
                                       const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onCleanUp(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onActivate(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Inactive() {}
        Inactive(const Inactive &other);
        Inactive &operator=(const Inactive &other);
    };

    class ShuttingDown : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        ShuttingDown() {}
        ShuttingDown(const Inactive &other);
        ShuttingDown &operator=(const Inactive &other);
    };

    class Unconfigured : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Unconfigured::*onTransition)(Node *,
                                           const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onConfigure(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Unconfigured() {}
        Unconfigured(const Unconfigured &other);
        Unconfigured &operator=(const Unconfigured &other);
    };

    class Unknown : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rstate_msgs::msg::NetworkState getNetworkState();
        rstate_msgs::srv::GetAvailableNetworkTransitions::Response getAvailableNetworkTransitions();
        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate_msgs::srv::NetworkTransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *,
                                    const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate_msgs::msg::NetworkTransitionFeedback>>);

    private:
        Unknown() {}
        Unknown(const Unknown &other);
        Unknown &operator=(const Unknown &other);
    };
} // namespace rstate

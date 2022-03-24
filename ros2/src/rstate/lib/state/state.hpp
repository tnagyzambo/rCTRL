#pragma once

#include <action/server.hpp>
#include <cmd/cmd.hpp>
#include <node.hpp>
#include <rstate/msg/transition_feedback.hpp>
#include <rstate/srv/transition_send_goal.hpp>
#include <sstream>

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    // enum class GoalResponse;
    // class GoalHandle<rstate::msg::TransitionFeedback>;
    class Node;

    enum class Transition { Configure, CleanUp, Activate, Deactivate, Arm, Disarm, Shutdown };

    enum class TransitionResult { Success, Cancelled, Failure };

    enum class ShutdownResult { Success, Failure };

    class State {
    public:
        virtual ~State() {}

        virtual void enter(Node *) = 0;

        virtual GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>) = 0;
        virtual CancelResponse handleCancel(Node *,
                                            const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>) = 0;
        virtual void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>) = 0;

        static TransitionResult executeCommands(Node *,
                                                std::vector<std::shared_ptr<CmdIface>>,
                                                const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        static void executeCommandsCancel(std::vector<std::shared_ptr<CmdIface>>,
                                          const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        static ShutdownResult executeCommandsShutdown(Node *,
                                                      std::vector<std::shared_ptr<CmdIface>>,
                                                      const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
    };

    class Activating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Activating() {}
        Activating(const Activating &other);
        Activating &operator=(const Activating &other);
    };

    class Active : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Active::*onTransition)(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onArm(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onDeactivate(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Active() {}
        Active(const Active &other);
        Active &operator=(const Active &other);
    };

    class Armed : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Armed::*onTransition)(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onDisarm(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Armed() {}
        Armed(const Armed &other);
        Armed &operator=(const Armed &other);
    };

    class Arming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Arming() {}
        Arming(const Arming &other);
        Arming &operator=(const Arming &other);
    };

    class CleaningUp : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        CleaningUp() {}
        CleaningUp(const CleaningUp &other);
        CleaningUp &operator=(const CleaningUp &other);
    };

    class Configuring : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Configuring() {}
        Configuring(const Configuring &other);
        Configuring &operator=(const Configuring &other);
    };

    class Deactivating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Deactivating() {}
        Deactivating(const Deactivating &other);
        Deactivating &operator=(const Deactivating &other);
    };

    class Disarming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Disarming() {}
        Disarming(const Disarming &other);
        Disarming &operator=(const Disarming &other);
    };

    class ErrorProcessing : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        ErrorProcessing() {}
        ErrorProcessing(const ErrorProcessing &other);
        ErrorProcessing &operator=(const ErrorProcessing &other);
    };

    class Finalized : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Finalized() {}
        Finalized(const Finalized &other);
        Finalized &operator=(const Finalized &other);
    };

    class Inactive : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Inactive::*onTransition)(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onCleanUp(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onActivate(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Inactive() {}
        Inactive(const Inactive &other);
        Inactive &operator=(const Inactive &other);
    };

    class ShuttingDown : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        ShuttingDown() {}
        ShuttingDown(const Inactive &other);
        ShuttingDown &operator=(const Inactive &other);
    };

    class Unconfigured : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Unconfigured::*onTransition)(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onConfigure(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void onShutdown(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

        GoalResponse handleGoal(Node *, std::shared_ptr<const rstate::srv::TransitionSendGoal::Request>);
        CancelResponse handleCancel(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);
        void handleAccepted(Node *, const std::shared_ptr<GoalHandle<rstate::msg::TransitionFeedback>>);

    private:
        Unconfigured() {}
        Unconfigured(const Unconfigured &other);
        Unconfigured &operator=(const Unconfigured &other);
    };
} // namespace rstate

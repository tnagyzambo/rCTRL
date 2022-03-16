#pragma once

#include <cmd/cmd.hpp>
#include <node.hpp>

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    class Node;

    enum class Transition { Configure, CleanUp, Activate, Deactivate, Arm, Disarm, Shutdown };

    enum class TransitionResult { Success, Cancelled, Failure };

    enum class ShutdownResult { Success, Failure };

    class State {
    public:
        virtual ~State() {}

        virtual void enter(Node *) = 0;

        virtual rclcpp_action::GoalResponse handleGoal(Node *,
                                                       const rclcpp_action::GoalUUID &,
                                                       std::shared_ptr<const rstate::action::Transition::Goal>) = 0;
        virtual rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>) = 0;
        virtual void handleAccepted(Node *,
                                    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>) = 0;

        static TransitionResult executeCommands(Node *,
                                                std::vector<std::shared_ptr<CmdIface>>,
                                                const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>>);
        static void executeCommandsCancel(std::vector<std::shared_ptr<CmdIface>>,
                                          const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>>);
        static ShutdownResult executeCommandsShutdown(
            Node *,
            std::vector<std::shared_ptr<CmdIface>>,
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::Transition>>);
    };

    class Activating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

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
                                     const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onArm(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onDeactivate(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onShutdown(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

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
                                    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onDisarm(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onShutdown(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Armed() {}
        Armed(const Armed &other);
        Armed &operator=(const Armed &other);
    };

    class Arming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Arming() {}
        Arming(const Arming &other);
        Arming &operator=(const Arming &other);
    };

    class CleaningUp : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        CleaningUp() {}
        CleaningUp(const CleaningUp &other);
        CleaningUp &operator=(const CleaningUp &other);
    };

    class Configuring : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Configuring() {}
        Configuring(const Configuring &other);
        Configuring &operator=(const Configuring &other);
    };

    class Deactivating : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Deactivating() {}
        Deactivating(const Deactivating &other);
        Deactivating &operator=(const Deactivating &other);
    };

    class Disarming : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Disarming() {}
        Disarming(const Disarming &other);
        Disarming &operator=(const Disarming &other);
    };

    class ErrorProcessing : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        ErrorProcessing() {}
        ErrorProcessing(const ErrorProcessing &other);
        ErrorProcessing &operator=(const ErrorProcessing &other);
    };

    class Finalized : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

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
                                       const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onCleanUp(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onActivate(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onShutdown(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Inactive() {}
        Inactive(const Inactive &other);
        Inactive &operator=(const Inactive &other);
    };

    class ShuttingDown : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        ShuttingDown() {}
        ShuttingDown(const Inactive &other);
        ShuttingDown &operator=(const Inactive &other);
    };

    class Unconfigured : public State {
    public:
        static State &getInstance();

        void enter(Node *);

        void (Unconfigured::*onTransition)(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onConfigure(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void onShutdown(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

        rclcpp_action::GoalResponse handleGoal(Node *,
                                               const rclcpp_action::GoalUUID &,
                                               std::shared_ptr<const rstate::action::Transition::Goal>);
        rclcpp_action::CancelResponse handleCancel(
            Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);
        void handleAccepted(Node *, const std::shared_ptr<rclcpp_action::ServerGoalHandle<rstate::action::Transition>>);

    private:
        Unconfigured() {}
        Unconfigured(const Unconfigured &other);
        Unconfigured &operator=(const Unconfigured &other);
    };
} // namespace rstate

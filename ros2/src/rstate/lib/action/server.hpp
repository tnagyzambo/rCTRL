#pragma once

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

// This is a scuffed reimplementation of ROS2 actions
// I needed to make this since rosbridge (the gui) currently does not support ROS2 actions
// It is also not possible to directly access the services and publishers that are composed to create a ROS2 action
// I tried to keep the public facing API as close to ROS2 actions as possible so that when the time comes to replace
// this code the transition is as smooth as possible The notable changes are that the goal UUID is missing from all
// server side function A simplification was made to remove the result request functionality from the action server This
// has no impact on the API to the rest of the rstate code but does change how action clients are supposed to interact
// with the server The request result code is instead published in the feedback topic

namespace rstate {
    enum class ResultCode { UNKNOWN = 1, SUCCEEDED = 1, CANCELED = 3, ABORTED = 4 };
    enum class GoalResponse { REJECT = 1, ACCEPT_AND_EXECUTE = 2, ACCEPT_AND_DEFER = 3 };
    enum class CancelResponse { REJECT = 1, ACCEPT = 2 };

    // Goal handle reimplements some of the functions in a ROS2 action goal handle
    // This is a shared reasource between threads
    // POSSIBILITY OF RACE CONDITIONS
    template <typename Feedback>
    class GoalHandle {
    public:
        GoalHandle(rclcpp_lifecycle::LifecycleNode *, std::string);

        bool is_canceling() { return this->isCanceling; }
        void publish_feedback(Feedback);

        bool isCanceling;

    private:
        typename rclcpp_lifecycle::LifecyclePublisher<Feedback>::SharedPtr feedbackPublisher;
    };

    template <typename Feedback>
    GoalHandle<Feedback>::GoalHandle(rclcpp_lifecycle::LifecycleNode *node, std::string topicBase) {
        this->isCanceling = false;

        std::string topicFeedback = topicBase + "/feedback";
        this->feedbackPublisher = node->create_publisher<Feedback>(topicFeedback, 10);
        this->feedbackPublisher->on_activate();
    }

    template <typename Feedback>
    void GoalHandle<Feedback>::publish_feedback(Feedback feedback) {
        this->feedbackPublisher->publish(feedback);
    }

    // Reimplement the action server API
    template <typename CancelGoal, typename SendGoal, typename Feedback>
    class ActionServer {
    public:
        ActionServer(rclcpp_lifecycle::LifecycleNode *,
                     std::string,
                     std::function<GoalResponse(std::shared_ptr<const typename SendGoal::Request>)>,
                     std::function<CancelResponse(const std::shared_ptr<GoalHandle<Feedback>>)>,
                     std::function<void(const std::shared_ptr<GoalHandle<Feedback>>)>);

    private:
        std::shared_ptr<GoalHandle<Feedback>> goalHandle;

        std::function<GoalResponse(std::shared_ptr<const typename SendGoal::Request>)> handleGoal;
        std::function<CancelResponse(typename std::shared_ptr<GoalHandle<Feedback>>)> handleCancel;
        std::function<void(typename std::shared_ptr<GoalHandle<Feedback>>)> handleAccepted;

        typename rclcpp::Service<CancelGoal>::SharedPtr serviceCancelGoal;
        typename rclcpp::Service<SendGoal>::SharedPtr serviceSendGoal;

        void serviceCancelGoalCallback(const std::shared_ptr<typename CancelGoal::Request>,
                                       std::shared_ptr<typename CancelGoal::Response>);
        void serviceSendGoalCallback(const std::shared_ptr<typename SendGoal::Request>,
                                     std::shared_ptr<typename SendGoal::Response>);
    };

    template <typename CancelGoal, typename SendGoal, typename Feedback>
    ActionServer<CancelGoal, SendGoal, Feedback>::ActionServer(
        rclcpp_lifecycle::LifecycleNode *node,
        std::string topicBase,
        std::function<GoalResponse(std::shared_ptr<const typename SendGoal::Request>)> handleGoal,
        std::function<CancelResponse(const typename std::shared_ptr<GoalHandle<Feedback>>)> handleCancel,
        std::function<void(const typename std::shared_ptr<GoalHandle<Feedback>>)> handleAccepted) {
        this->goalHandle = std::make_shared<GoalHandle<Feedback>>(node, topicBase);

        this->handleGoal = handleGoal;
        this->handleAccepted = handleAccepted;
        this->handleCancel = handleCancel;

        std::string topicCancelGoal = topicBase + "/cancel_goal";
        this->serviceCancelGoal = node->create_service<CancelGoal>(
            topicCancelGoal,
            std::bind(&ActionServer::serviceCancelGoalCallback, this, std::placeholders::_1, std::placeholders::_2));

        std::string topicSendGoal = topicBase + "/send_goal";
        this->serviceSendGoal = node->create_service<SendGoal>(
            topicSendGoal,
            std::bind(&ActionServer::serviceSendGoalCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Wrapping function to bind service request to handleCancel
    template <typename CancleGoal, typename SendGoal, typename Feedback>
    void ActionServer<CancleGoal, SendGoal, Feedback>::serviceCancelGoalCallback(
        const std::shared_ptr<typename CancleGoal::Request> request,
        std::shared_ptr<typename CancleGoal::Response> response) {
        (void)request;
        CancelResponse cancel_response = this->handleCancel(this->goalHandle);

        if (cancel_response == CancelResponse::ACCEPT) {
            this->goalHandle->isCanceling = true;
        }

        response->cancel_response = (uint)cancel_response;
        return;
    }

    // Wrapping function bind service request to handleGoal and handleAccepted
    template <typename CancleGoal, typename SendGoal, typename Feedback>
    void ActionServer<CancleGoal, SendGoal, Feedback>::serviceSendGoalCallback(
        const std::shared_ptr<typename SendGoal::Request> request, std::shared_ptr<typename SendGoal::Response> response) {
        GoalResponse goal_response = this->handleGoal(request);

        if (goal_response == GoalResponse::ACCEPT_AND_EXECUTE) {
            this->handleAccepted(this->goalHandle);
        }

        response->goal_response = (uint)goal_response;
        return;
    }
} // namespace rstate

#pragma once

#include <chrono>
#include <cmd/cmd.hpp>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <string>
#include <util/except.hpp>
#include <util/util.hpp>

using namespace std::chrono_literals;

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    class Node;

    template <typename T>
    class CmdService : public CmdIface {
    public:
        CmdService(std::shared_ptr<rclcpp::Client<T>>, toml::node_view<toml::node>, bool = false);

        void execute();
        void cancel();

    protected:
        std::shared_ptr<rclcpp::Client<T>> client;
        std::shared_ptr<typename T::Request> request;
        std::shared_ptr<typename T::Response> response;

        std::shared_ptr<typename T::Request> requestCancel;
        std::shared_ptr<typename T::Response> responseCancel;

        std::chrono::milliseconds waitForServiceTimeOut;
        std::chrono::milliseconds requestTimeOut;

        void createRequest(toml::node_view<toml::node>);
        void createRequestCancel(toml::node_view<toml::node>);
        void sendRequest(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>);
        void waitForService();
        void waitForFuture(std::shared_future<typename rclcpp::Client<T>::SharedResponse>);
        void compareResponse(std::shared_ptr<typename T::Response>, std::shared_ptr<typename T::Response>);
    };

    template <typename T>
    CmdService<T>::CmdService(std::shared_ptr<rclcpp::Client<T>> client, toml::node_view<toml::node> toml, bool allowCancel)
        : CmdIface(toml, allowCancel) {
        this->client = client;
        this->createRequest(toml);

        if (allowCancel) {
            this->createRequestCancel(toml);
        }

        this->waitForServiceTimeOut =
            std::chrono::milliseconds(util::toml::getTomlEntryByKey<uint>(toml, "timeout_wait_for_srv"));
        this->requestTimeOut = std::chrono::milliseconds(util::toml::getTomlEntryByKey<uint>(toml, "timeout_request"));
    }

    template <typename T>
    void CmdService<T>::createRequest(toml::node_view<toml::node> toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "Service: " << util::toml::getTomlEntryByKey<std::string>(toml, "service") << "\n";
        error << "Section: " << toml << "\n";

        throw except::config_parse_error(error.str());
    }

    template <typename T>
    void CmdService<T>::createRequestCancel(toml::node_view<toml::node> toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "TOML: " << toml << "\n";

        throw except::config_parse_error(error.str());
    }

    template <typename T>
    void CmdService<T>::execute() {
        this->sendRequest(this->request, this->response);
    }

    template <typename T>
    void CmdService<T>::cancel() {
        this->sendRequest(this->requestCancel, this->responseCancel);
    }

    template <typename T>
    void CmdService<T>::sendRequest(std::shared_ptr<typename T::Request> request,
                                    std::shared_ptr<typename T::Response> expectedResponse) {
        this->waitForService();

        auto future = this->client->async_send_request(request);
        this->waitForFuture(future);

        auto response = future.get();
        this->compareResponse(response, expectedResponse);
    }

    template <typename T>
    void CmdService<T>::waitForService() {
        auto pollRate = 10ms;

        bool serviceStatus = false;
        auto serviceWaitTime = 0ms;
        do {
            serviceStatus = this->client->wait_for_service(pollRate);
            if (serviceStatus) {
                return;
            }
            serviceWaitTime += pollRate;
        } while (serviceWaitTime <= this->waitForServiceTimeOut);

        std::stringstream error;

        error << "Timed out, service not available!\n";
        error << "Service: " << this->client->get_service_name() << "\n";

        throw except::cmd_service_eror(error.str());
    }

    template <typename T>
    void CmdService<T>::waitForFuture(std::shared_future<typename rclcpp::Client<T>::SharedResponse> future) {
        auto pollRate = 10ms;

        std::future_status futureStatus;
        auto futureWaitTime = 0ms;
        do {
            futureStatus = future.wait_for(pollRate);
            if (futureStatus == std::future_status::ready) {
                return;
            }
            futureWaitTime += pollRate;
        } while (futureStatus != std::future_status::ready);

        std::stringstream error;

        error << "Timed out while waiting for response from service!\n";
        error << "Service: " << this->client->get_service_name() << "\n";

        throw except::cmd_service_eror(error.str());
    }

    template <typename T>
    void CmdService<T>::compareResponse(std::shared_ptr<typename T::Response> response,
                                        std::shared_ptr<typename T::Response> expectedResponse) {
        (void)response;
        (void)expectedResponse;
        std::stringstream error;

        error << "No template specialization found for compareResponse()!\n";

        throw except::cmd_service_eror(error.str());
    }

    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::createRequest(toml::node_view<toml::node>);

    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::createRequestCancel(toml::node_view<toml::node>);

    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::compareResponse(
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>,
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>);
} // namespace rstate

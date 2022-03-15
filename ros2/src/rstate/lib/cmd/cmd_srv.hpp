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

    // Base non-cancelable service command
    template <typename T>
    class CmdService : public CmdIface {
    public:
        CmdService(std::shared_ptr<rclcpp::Client<T>>, toml::table);

        void execute();
        void cancel();

    protected:
        std::shared_ptr<rclcpp::Client<T>> client;
        std::shared_ptr<typename T::Request> request;
        std::shared_ptr<typename T::Response> response;

        std::chrono::milliseconds waitForServiceTimeOut;
        std::chrono::milliseconds requestTimeOut;

        void createRequest(toml::table);
        void sendRequest(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>);
        void waitForService();
        void waitForFuture(std::shared_future<typename rclcpp::Client<T>::SharedResponse>);
        void compareResponse(std::shared_ptr<typename T::Response>, std::shared_ptr<typename T::Response>);
    };

    template <typename T>
    CmdService<T>::CmdService(std::shared_ptr<rclcpp::Client<T>> client, toml::table toml) : CmdIface(toml) {
        this->client = client;
        this->createRequest(toml);

        this->waitForServiceTimeOut =
            std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_wait_for_srv"));
        this->requestTimeOut = std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_request"));
    }

    template <typename T>
    void CmdService<T>::createRequest(toml::table toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "Service: " << util::getTomlEntryByKey<std::string>(toml, "service") << "\n";
        error << "Section: " << toml << "\n";

        throw except::config_parse_error(error.str());
    }

    template <typename T>
    void CmdService<T>::execute() {
        this->sendRequest(this->request, this->response);
    }

    template <typename T>
    void CmdService<T>::cancel() {
        std::stringstream error;

        error << "Attempted to cancel non cancelable command!\n";
        error << "Command ID: " << this->id << "\n";

        throw except::cmd_service_eror(error.str());
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

    // Extend and pratially overide base non-cancelable service command to allow cancel
    template <typename T>
    class CmdServiceCancelable : public CmdService<T> {
    public:
        CmdServiceCancelable(std::shared_ptr<rclcpp::Client<T>>, toml::table);

        void cancel();

    private:
        std::shared_ptr<typename T::Request> requestCancel;
        std::shared_ptr<typename T::Response> responseCancel;

        void createRequestCancel(toml::table);
    };

    template <typename T>
    CmdServiceCancelable<T>::CmdServiceCancelable(std::shared_ptr<rclcpp::Client<T>> client, toml::table toml)
        : CmdService<T>(client, toml) {
        this->createRequestCancel(toml);
    }

    template <typename T>
    void CmdServiceCancelable<T>::createRequestCancel(toml::table toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "Service: " << util::getTomlEntryByKey<std::string>(toml, "service") << "\n";
        error << "Section: " << toml << "\n";

        throw except::config_parse_error(error.str());
    }

    template <typename T>
    void CmdServiceCancelable<T>::cancel() {
        this->sendRequest(this->requestCancel, this->responseCancel);
    }

} // namespace rstate

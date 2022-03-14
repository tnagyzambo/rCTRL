#pragma once

#include <any>
#include <chrono>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rstate_except.hpp>
#include <rstate_util.hpp>
#include <stdexcept>
#include <string>
#include <toml++/toml.h>

using namespace std::chrono_literals;

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    class Node;

    // class WaitCommand : public Command {
    // public:
    //     float waitTime;
    // };

    class CmdIface {
    public:
        CmdIface(toml::table toml) { this->id = util::getTomlEntryByKey<uint>(toml, "id"); }
        virtual ~CmdIface() {}

        uint id;

        virtual void execute() = 0;
        virtual void cancel() = 0;
    };

    template <typename T>
    class Cmd : public CmdIface {
    public:
        Cmd(std::shared_ptr<rclcpp::Client<T>>, toml::table);

        void execute();
        void cancel();

    private:
        std::shared_ptr<rclcpp::Client<T>> client;
        std::shared_ptr<typename T::Request> request;
        std::shared_ptr<typename T::Request> requestCancel;
        std::shared_ptr<typename T::Response> response;
        std::shared_ptr<typename T::Response> responseCancel;

        std::chrono::milliseconds waitForServiceTimeOut;
        std::chrono::milliseconds requestTimeOut;

        void createRequests(toml::table);
        void sendRequest(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>);
        void waitForService();
        void waitForFuture(std::shared_future<typename rclcpp::Client<T>::SharedResponse>);
        void compareResponse(std::shared_ptr<typename T::Response>, std::shared_ptr<typename T::Response>);
    };

    template <typename T>
    Cmd<T>::Cmd(std::shared_ptr<rclcpp::Client<T>> client, toml::table toml) : CmdIface(toml) {
        this->client = client;
        this->createRequests(toml);

        this->waitForServiceTimeOut =
            std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_wait_for_srv"));
        this->requestTimeOut = std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_request"));
    }

    template <typename T>
    void Cmd<T>::createRequests(toml::table toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "Service: " << util::getTomlEntryByKey<std::string>(toml, "service") << "\n";
        error << "Section: " << toml << "\n";

        throw except::config_parse_error(error.str());
    }

    template <typename T>
    void Cmd<T>::execute() {
        sendRequest(this->request, this->response);
    }

    template <typename T>
    void Cmd<T>::cancel() {
        sendRequest(this->requestCancel, this->responseCancel);
    }

    template <typename T>
    void Cmd<T>::sendRequest(std::shared_ptr<typename T::Request> request,
                             std::shared_ptr<typename T::Response> expectedResponse) {
        waitForService();

        auto future = this->client->async_send_request(request);
        waitForFuture(future);

        auto response = future.get();
        compareResponse(response, expectedResponse);
    }

    template <typename T>
    void Cmd<T>::waitForService() {
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
    void Cmd<T>::waitForFuture(std::shared_future<typename rclcpp::Client<T>::SharedResponse> future) {
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
    void Cmd<T>::compareResponse(std::shared_ptr<typename T::Response> response,
                                 std::shared_ptr<typename T::Response> expectedResponse) {
        (void)response;
        (void)expectedResponse;
        std::stringstream error;

        error << "No template specialization found for compareResponse()!\n";

        throw except::cmd_service_eror(error.str());
    }

    // Interface for templated CmdClient
    class CmdServiceIface {
    public:
        virtual ~CmdServiceIface() {}
    };

    // Class to manage construction of all Cmd's of a shared type
    template <typename T>
    class CmdService : public CmdServiceIface {
    public:
        CmdService(toml::table,
                   std::string,
                   std::shared_ptr<rclcpp::Client<T>>,
                   std::map<uint, std::shared_ptr<CmdIface>> &);

    private:
        std::string serviceName;
        std::shared_ptr<rclcpp::Client<T>> client;
        std::vector<std::shared_ptr<Cmd<T>>> cmds;

        std::vector<std::shared_ptr<Cmd<T>>> createCmds(toml::table,
                                                        std::shared_ptr<rclcpp::Client<T>>,
                                                        std::map<uint, std::shared_ptr<CmdIface>> &);
    };

    template <typename T>
    CmdService<T>::CmdService(toml::table toml,
                              std::string serviceName,
                              std::shared_ptr<rclcpp::Client<T>> client,
                              std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        this->serviceName = serviceName;
        this->client = client;
        this->cmds = createCmds(toml, this->client, cmdMap);
    }

    template <typename T>
    std::vector<std::shared_ptr<Cmd<T>>> CmdService<T>::createCmds(toml::table toml,
                                                                   std::shared_ptr<rclcpp::Client<T>> client,
                                                                   std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        std::vector<std::shared_ptr<Cmd<T>>> cmds;
        for (auto key : {"configure", "cleanup", "activate", "deactivate", "arm", "disarm"}) {
            auto elems = util::getCmdsAsArray(*(toml[key]).as_table(), "cmd_service");
            if (elems != nullptr) {
                for (toml::node &elem : *elems) {
                    if (util::getServiceName(*elem.as_table()) == this->serviceName) {
                        auto cmd = Cmd(client, *elem.as_table());

                        // Register base interface of cmd in cmdMap with unique ID
                        auto insertResult = cmdMap.insert(
                            std::pair<uint, std::shared_ptr<CmdIface>>(cmd.id, std::make_shared<Cmd<T>>(cmd)));

                        // This should never throw, but just for safety
                        if (insertResult.second == false) {
                            std::stringstream error;

                            error << "ID collision during cmdMap insertion!\n";
                            error << "ID: " << cmd.id << "\n";
                            error << "Section: " << toml << "\n";

                            throw except::config_parse_error(error.str());
                        }

                        cmds.push_back(std::make_shared<Cmd<T>>(cmd));
                    }
                }
            }
        }

        return cmds;
    }
} // namespace rstate

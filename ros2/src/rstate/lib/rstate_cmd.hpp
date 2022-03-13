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

        virtual bool execute() = 0;
    };

    template <typename T>
    class Cmd : public CmdIface {
    public:
        Cmd(rclcpp::Node *, std::shared_ptr<rclcpp::Client<T>>, toml::table);

        bool execute();

    private:
        rclcpp::Node *node;
        std::shared_ptr<rclcpp::Client<T>> client;
        std::shared_ptr<typename T::Request> request;

        std::chrono::milliseconds waitForServiceTimeOut;
        std::chrono::milliseconds requestTimeOut;

        std::shared_ptr<typename T::Request> createRequest(toml::table);
    };

    template <typename T>
    Cmd<T>::Cmd(rclcpp::Node *node, std::shared_ptr<rclcpp::Client<T>> client, toml::table toml) : CmdIface(toml) {
        this->node = node;
        this->client = client;
        this->request = this->createRequest(toml);
        this->waitForServiceTimeOut =
            std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_wait_for_srv"));
        this->requestTimeOut = std::chrono::milliseconds(util::getTomlEntryByKey<uint>(toml, "timeout_request"));
    }

    template <typename T>
    std::shared_ptr<typename T::Request> Cmd<T>::createRequest(toml::table toml) {
        std::stringstream error;

        error << "No template specialication found for service type!\n";
        error << "Service: " << util::getTomlEntryByKey<std::string>(toml, "service") << "\n";
        error << "Section: " << toml << "\n";

        throw std::runtime_error(error.str());
    }

    template <typename T>
    bool Cmd<T>::execute() {
        std::cout << "hello\n";

        if (!this->client->wait_for_service(waitForServiceTimeOut)) {
            RCLCPP_ERROR(this->node->get_logger(), "Service '%s' is not available", this->client->get_service_name());
            return false;
        }

        auto future = this->client->async_send_request(this->request);

        // change this its wrong
        std::future_status status;
        do {
            switch (status = future.wait_for(requestTimeOut); status) {
            case std::future_status::deferred:
                std::cout << "deferred\n";
                break;
            case std::future_status::timeout:
                std::cout << "timeout\n";
                return false;
            case std::future_status::ready:
                std::cout << "ready!\n";
                break;
            }
        } while (status != std::future_status::ready);

        auto test = future.get();

        // check against expected

        return true;
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
        CmdService(rclcpp::Node *,
                   toml::table,
                   std::string,
                   std::shared_ptr<rclcpp::Client<T>>,
                   std::map<uint, std::shared_ptr<CmdIface>> &);

    private:
        rclcpp::Node *node;
        std::string serviceName;
        std::shared_ptr<rclcpp::Client<T>> client;
        std::vector<std::shared_ptr<Cmd<T>>> cmds;

        std::vector<std::shared_ptr<Cmd<T>>> createCmds(rclcpp::Node *,
                                                        toml::table,
                                                        std::shared_ptr<rclcpp::Client<T>>,
                                                        std::map<uint, std::shared_ptr<CmdIface>> &);
        // std::shared_ptr<Cmd<T>> createCmd(toml::table, std::shared_ptr<rclcpp::Client<T>>);
    };

    template <typename T>
    CmdService<T>::CmdService(rclcpp::Node *node,
                              toml::table toml,
                              std::string serviceName,
                              std::shared_ptr<rclcpp::Client<T>> client,
                              std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        this->node = node;
        this->serviceName = serviceName;
        this->client = client;
        this->cmds = createCmds(this->node, toml, this->client, cmdMap);
    }

    template <typename T>
    std::vector<std::shared_ptr<Cmd<T>>> CmdService<T>::createCmds(rclcpp::Node *node,
                                                                   toml::table toml,
                                                                   std::shared_ptr<rclcpp::Client<T>> client,
                                                                   std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        std::vector<std::shared_ptr<Cmd<T>>> cmds;
        for (auto key : {"configure", "cleanup", "activate", "deactivate", "arm", "disarm"}) {
            auto elems = util::getCmdsAsArray(*(toml[key]).as_table(), "cmd_service");
            if (elems != nullptr) {
                for (toml::node &elem : *elems) {
                    if (util::getServiceName(*elem.as_table()) == this->serviceName) {
                        auto cmd = Cmd(node, client, *elem.as_table());

                        // Register base interface of cmd in cmdMap with unique ID
                        auto insertResult = cmdMap.insert(
                            std::pair<uint, std::shared_ptr<CmdIface>>(cmd.id, std::make_shared<Cmd<T>>(cmd)));

                        // This should never throw, but just for safety
                        if (insertResult.second == false) {
                            std::stringstream error;

                            error << "ID collision during cmdMap insertion!\n";
                            error << "ID: " << cmd.id << "\n";
                            error << "Section: " << toml << "\n";

                            throw std::runtime_error(error.str());
                        }

                        cmds.push_back(std::make_shared<Cmd<T>>(cmd));
                        std::cout << "accepted: " << (*elem.as_table())["id"] << "\n";
                    } else {
                        std::cout << "rejected: " << (*elem.as_table())["id"] << "\n";
                    }
                }
            }
        }

        return cmds;
    }
} // namespace rstate

#pragma once

#include <cmd/cmd.hpp>
#include <cmd/cmd_srv.hpp>
#include <cmd/cmd_srv_client.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rstate/action/transition.hpp>
#include <state/state.hpp>
#include <string>
#include <toml++/toml.h>
#include <util/util.hpp>
#include <vector>

namespace rstate {
    // Forward declaration to resolve circular dependency/include
    class State;

    class Node : public rclcpp::Node {
    public:
        explicit Node();

        rclcpp_action::Server<rstate::action::Transition>::SharedPtr actionServer;

        // Vectors to hold all cmds to be exectuted on transitions in order
        std::vector<std::shared_ptr<CmdIface>> cmdsOnConfigure;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnCleanUp;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnActivate;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnDeactivate;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnArm;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnDisarm;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownUnconfigured;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownConfigured;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownActive;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownArmed;
        std::map<std::string, std::vector<std::shared_ptr<CmdIface>> *> transitionMap;

        // Map to hold all commands defined in config.toml
        // Key value is a uniquely assined ID
        std::map<uint, std::shared_ptr<CmdIface>> cmdMap;

        // Vec to hold CmdClient<T>'s
        // Not used directly, only to keep the stored classes from going out of scope
        std::vector<std::shared_ptr<CmdServiceClientIface>> cmdSrvClients;

        // Map with service names as keys and constructed ROS2 clients as values
        // Used to prevent registering multiple clients to the same service
        // Also keeps the clients from going our of scope
        std::map<std::string, std::shared_ptr<rclcpp::ClientBase>> clientMap;
        // std::vector<std::shared_ptr<CmdClientIface>> onConfigureCommands;

        State *getState();
        void setState(State &);

        void assignCmdIds(toml::table &);
        void createCmds(toml::table &);
        void registerCmds(toml::table &);
        void createCmdService(toml::table &, toml::table &);

    private:
        State *currentState;
    };

    static const std::vector<const char *> configSections = {"configure",
                                                             "cleanup",
                                                             "activate",
                                                             "deactivate",
                                                             "arm",
                                                             "disarm",
                                                             "shutdown_unconfigured",
                                                             "shutdown_inactive",
                                                             "shutdown_active",
                                                             "shutdown_armed"};

    // Map from 'service' definition in the .toml to a int used in the switch case of createCmdClients()
    // These two MUST match or there will be runtime errors
    static const std::unordered_map<std::string, int> srvTypeMap{{"change_state", 1}, {"get_state", 2}};
} // namespace rstate

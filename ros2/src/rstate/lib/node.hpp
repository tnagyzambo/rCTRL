#pragma once

#include <action/server.hpp>
#include <cmd/cmd.hpp>
#include <cmd/service.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rstate_msgs/msg/network_state.hpp>
#include <rstate_msgs/msg/network_transition_description.hpp>
#include <rstate_msgs/msg/network_transition_event.hpp>
#include <rstate_msgs/msg/network_transition_feedback.hpp>
#include <rstate_msgs/srv/get_available_network_states.hpp>
#include <rstate_msgs/srv/get_available_network_transitions.hpp>
#include <rstate_msgs/srv/get_network_state.hpp>
#include <rstate_msgs/srv/network_transition_cancel_goal.hpp>
#include <rstate_msgs/srv/network_transition_send_goal.hpp>
#include <rutil/fmt.hpp>
#include <state/state.hpp>
#include <string>
#include <toml++/toml.h>
#include <toml++/toml_node.h>
#include <toml++/toml_node_view.h>
#include <util/util.hpp>
#include <vector>

namespace rstate {
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // Forward declaration to resolve circular dependency/include
    class State;
    enum class NetworkStateEnum;
    enum class NetworkTransitionEnum;

    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

        // Vectors to hold all cmds to be exectuted on transitions in order
        std::vector<std::shared_ptr<CmdIface>> cmdsOnConfigure;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnCleanUp;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnActivate;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnDeactivate;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownUnconfigured;
        std::vector<std::shared_ptr<CmdIface>> cmdOnShutdownInactive;
        std::vector<std::shared_ptr<CmdIface>> cmdsOnShutdownActive;

        void setState(State &);
        void publishNetworkTransitionEvent(NetworkTransitionEnum, NetworkStateEnum, NetworkStateEnum);

    private:
        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        State *currentState;

        rclcpp_lifecycle::LifecyclePublisher<rstate_msgs::msg::NetworkTransitionEvent>::SharedPtr publisherNetworkTransitionEvent;

        rclcpp::Service<rstate_msgs::srv::GetNetworkState>::SharedPtr serviceGetNetworkState;
        rclcpp::Service<rstate_msgs::srv::GetAvailableNetworkTransitions>::SharedPtr serviceGetAvailableNetworkTransistions;
        rclcpp::Service<rstate_msgs::srv::GetAvailableNetworkStates>::SharedPtr serviceGetAvailableNetworkStates;

        void serviceGetNetworkStateCallback(const std::shared_ptr<rstate_msgs::srv::GetNetworkState::Request>,
                                            const std::shared_ptr<rstate_msgs::srv::GetNetworkState::Response>);
        void serviceGetAvailableNetworkTransistionsCallback(
            const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkTransitions::Request>,
            const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkTransitions::Response>);
        void serviceGetNetworkAvailableStatesCallback(
            const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkStates::Request>,
            const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkStates::Response>);

        std::shared_ptr<ActionServer<rstate_msgs::srv::NetworkTransitionCancelGoal,
                                     rstate_msgs::srv::NetworkTransitionSendGoal,
                                     rstate_msgs::msg::NetworkTransitionFeedback>>
            actionServer;

        // Map to hold exectution buffers for transitions
        std::map<const char *, std::vector<std::shared_ptr<CmdIface>> *> transitionMap;

        // Map to hold command constructors based on type
        std::unordered_map<const char *, std::function<std::shared_ptr<CmdIface>(toml::node_view<toml::node>, bool)>> cmdTypeMap;

        // Map with service names as keys and constructed ROS2 clients as values
        // Used to prevent registering multiple clients to the same service
        // Also keeps the clients from going our of scope
        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> cmdServiceClientMapChangeState;
        std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> cmdServiceClientMapGetState;

        void readConfig(toml::table);
        std::shared_ptr<CmdIface> createCmd(toml::node_view<toml::node>, bool);
        std::shared_ptr<CmdIface> createCmdService(toml::node_view<toml::node>, bool);
        template <typename T>
        std::shared_ptr<CmdIface> createCmdServiceType(toml::node_view<toml::node>,
                                                       bool,
                                                       std::map<std::string, std::shared_ptr<rclcpp::Client<T>>> &);

        void deleteAllPointers();
    };

    // Map from 'service' definition in the .toml to a int used in the switch case of createCmdClients()
    // These two MUST match or there will be runtime errors
    static const std::unordered_map<std::string, int> cmdServiceTypeMap{{"change_state", 1}, {"get_state", 2}};

    // Templated helper to create a cmd.service of a particular type
    template <typename T>
    std::shared_ptr<CmdIface> Node::createCmdServiceType(
        toml::node_view<toml::node> cmdServiceView,
        bool allowCancel,
        std::map<std::string, std::shared_ptr<rclcpp::Client<T>>> &clientMap) {
        // Get the 'node/service' name registered in the toml
        std::string serviceName = util::toml::getServiceName(cmdServiceView);

        // If the service name has not been registered in the clientMap, create Client and CmdClients
        // Prevents registering two clients to the same service
        if (!clientMap.count(serviceName)) {
            std::shared_ptr<rclcpp::Client<T>> client = this->create_client<T>(serviceName);
            clientMap.emplace(serviceName, client);
        }

        // This should never throw, but just in case
        auto clientFindResult = clientMap.find(serviceName);
        if (clientFindResult == clientMap.end()) {
            std::stringstream error;

            error << "Attempted to create service command with no existing client!\n";
            error << "Service: " << serviceName << "\n";
            error << "TOML: " << cmdServiceView << "\n";

            throw rstate::except::config_parse_error(error.str());
        }

        auto client = clientFindResult->second;

        std::shared_ptr<CmdService<T>> cmdPtr;

        auto cmd = CmdService<T>(client, cmdServiceView, allowCancel);
        cmdPtr = std::make_shared<CmdService<T>>(cmd);

        return cmdPtr;
    }
} // namespace rstate

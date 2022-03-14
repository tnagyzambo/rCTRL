#include <rclcpp/logging.hpp>
#include <rstate_node.hpp>
#include <stdexcept>

namespace rstate {
    Node::Node() : rclcpp::Node("rstate") {
        this->transitionMap = {
            {"configure", &this->cmdsOnConfigure},
            {"cleanup", &this->cmdsOnCleanUp},
            {"activate", &this->cmdsOnActivate},
            {"deactivate", &this->cmdsOnDeactivate},
            {"arm", &this->cmdsOnArm},
            {"disarm", &this->cmdsOnDisarm},
            {"shutdown.unconfigured", &this->cmdsOnShutdownUnconfigured},
            {"shutdown.configured", &this->cmdsOnShutdownConfigured},
            {"shutdown.active", &this->cmdsOnShutdownActive},
            {"shutdown.armed", &this->cmdsOnShutdownArmed},
        };

        // THESE CAN ERROR
        // FAIL CONFIGURATION OF NODE ON ERROR
        {
            toml::table toml = toml::parse_file("/workspaces/rCTRL/ros2/src/rstate/config.toml");

            // Assign unique ID's for all commands
            assignCmdIds(toml);

            createCmds(toml);

            registerCmds(toml);
        }

        // Set initial network state
        this->setState(Unconfigured::getInstance());

        // Bind action server call backs to State interface method
        // Provide a reference to the State currently held by the rstate Node as an execution context
        this->actionServer = rclcpp_action::create_server<action::Transition>(
            this,
            "rstate/transition",
            std::bind(&State::handleGoal, std::ref(this->currentState), this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&State::handleCancel, std::ref(this->currentState), this, std::placeholders::_1),
            std::bind(&State::handleAccepted, std::ref(this->currentState), this, std::placeholders::_1));
    }

    // Get current node state
    State *Node::getState() { return this->currentState; }

    // Assign the state of the node and trigger enter() event
    void Node::setState(State &newState) {
        // this->currentState->exit(this); (this can be defined for all states and used if needed)
        this->currentState = &newState;
        this->currentState->enter(this);
    }

    // Iterate through all sections of the config.toml
    // Generate and append unique ID's to all commands
    void Node::assignCmdIds(toml::table &toml) {
        uint id = 0;

        for (auto section : {"configure", "cleanup", "activate", "deactivate", "arm", "disarm"}) {
            for (auto cmdType : {"cmd_service", "cmd_wait"}) {
                auto cmds = util::getCmdsAsArray(*(toml[section]).as_table(), cmdType);
                if (cmds != nullptr) {
                    for (toml::node &cmd : *cmds) {
                        cmd.as_table()->insert_or_assign("id", id);
                        id++;
                    }
                }
            }
        }
    }

    // Create all commands found in config.toml
    void Node::createCmds(toml::table &toml) {
        // Iterate through all sections of the config.toml
        // If section contains commands, construct them and register their ID's in the relevent execution tables
        for (auto section : {"configure", "cleanup", "activate", "deactivate", "arm", "disarm"}) {
            // Construct all service commands
            RCLCPP_INFO(this->get_logger(), "In TOML section: '%s', creating all 'cmd_service'", section);
            auto cmds = util::getCmdsAsArray(*(toml[section]).as_table(), "cmd_service");
            if (cmds != nullptr) {
                for (toml::node &cmd : *cmds) {
                    createCmdService(toml, *cmd.as_table());
                }
            }
        }
    }

    // Iterate through all sections of the config.toml
    // Register all commands by their ID to the appropriate cmd queue
    void Node::registerCmds(toml::table &toml) {
        for (auto section : {"configure", "cleanup", "activate", "deactivate", "arm", "disarm"}) {
            auto transitionFindResult = this->transitionMap.find(section);

            if (transitionFindResult == this->transitionMap.end()) {
                throw std::runtime_error("no transition found");
            }

            auto transition = transitionFindResult->second;

            auto cmds = util::getCmdsAsArray(*(toml[section]).as_table(), "cmd_service");
            if (cmds != nullptr) {
                for (toml::node &cmd : *cmds) {
                    uint id = util::getTomlEntryByKey<uint>(*cmd.as_table(), "id");
                    auto cmdFindResult = this->cmdMap.find(id);

                    if (cmdFindResult == this->cmdMap.end()) {
                        throw std::runtime_error("no cmd found");
                    }

                    transition->push_back(cmdFindResult->second);
                }
            }
        }
    }

    // Create all 'cmd_service's found in config.toml
    // These are commands that are meant to be executed against ROS2 services
    // They require an approriate ROS2 client and msg's to be constructed in addition to registering their
    // exec() method to the relevent execution table
    void Node::createCmdService(toml::table &toml, toml::table &cmd) {
        // Get the 'node/service' name registered in the toml
        std::string serviceName = util::getServiceName(cmd);

        // If the service name has not been registered in the clientMap, create Client and CmdClients
        // Prevents registering two clients to the same service
        if (!clientMap.count(serviceName)) {
            // ROS2 messages do not inherit from a common interface
            // This results in us having to define create a very heirarchical structure since polymorphism is not
            // possible Get service type declared in .toml to switch on the template arguments for the Clients and
            // CmdClients This switch case MUST match the definition found in 'srvTypeMap' or there will be runtime
            // errors Replacing this switch with a map is not possible due to the lack of a common inteface
            std::string serviceType = util::getTomlEntryByKey<std::string>(cmd, "service");

            try {
                switch (srvTypeMap.at(serviceType)) {
                case 1: {
                    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(serviceName);
                    this->clientMap[serviceName] = client;
                    auto cmdClient = CmdService(toml, serviceName, client, this->cmdMap);
                    this->cmdSrvClients.push_back(
                        std::make_shared<CmdService<lifecycle_msgs::srv::ChangeState>>(cmdClient));
                } break;
                case 2: {
                    auto client = this->create_client<lifecycle_msgs::srv::GetState>(serviceName);
                    this->clientMap[serviceName] = client;
                    auto cmdClient = CmdService(toml, serviceName, client, this->cmdMap);
                    this->cmdSrvClients.push_back(std::make_shared<CmdService<lifecycle_msgs::srv::GetState>>(cmdClient));
                } break;
                default:
                    throw std::out_of_range("ROS2 service type defined in 'srvTypeMap' but not in switch case");
                };
            } catch (const std::out_of_range &e) {
                std::stringstream error;

                error << "Service in toml section has no known type!\n";
                error << "Service: " << serviceType << "\n";
                error << "Section: " << cmd << "\n";
                error << "Error: " << e.what() << "\n";

                throw rstate::except::config_parse_error(error.str());
            }
        }
    }
} // namespace rstate

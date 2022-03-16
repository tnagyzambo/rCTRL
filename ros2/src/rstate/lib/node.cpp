#include <node.hpp>
#include <rclcpp/logging.hpp>
#include <toml++/toml_node_view.h>

namespace rstate {
    Node::Node() : rclcpp::Node("rstate") {
        // Map toml sections to vectors that will store commands to execute in order
        this->transitionMap = {
            {"configure", &this->cmdsOnConfigure},
            {"cleanup", &this->cmdsOnCleanUp},
            {"activate", &this->cmdsOnActivate},
            {"deactivate", &this->cmdsOnDeactivate},
            {"arm", &this->cmdsOnArm},
            {"disarm", &this->cmdsOnDisarm},
            {"shutdown_unconfigured", &this->cmdsOnShutdownUnconfigured},
            {"shutdown_inactive", &this->cmdOnShutdownInactive},
            {"shutdown_active", &this->cmdsOnShutdownActive},
            {"shutdown_armed", &this->cmdsOnShutdownArmed},
        };

        // Place member functions needed to construct command type into map with lambda functions
        this->cmdTypeMap = {{"service", [&](toml::node_view<toml::node> cmdServiceView, bool allowCancel) {
                                 return this->createCmdService(cmdServiceView, allowCancel);
                             }}};

        // THESE CAN ERROR
        // FAIL CONFIGURATION OF RSTATE NODE ON ERROR
        {
            toml::table toml = toml::parse_file("/workspaces/rCTRL/ros2/src/rstate/config.toml");

            readConfig(toml);
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

    // Parse .toml and create all commands
    void Node::readConfig(toml::table toml) {
        auto tomlView = toml::node_view(toml);

        for (auto transition : transitionMap) {
            auto transitionName = transition.first;
            auto transitionCmdVector = transition.second;

            // Get view of valid toml table
            auto transitionView = util::toml::viewOfTable(tomlView, transitionName);

            // util::toml::viewOfArray will throw if there is no commands in a section
            // We need to catch the execption and skip assignment of ID's in this case
            toml::node_view<toml::node> cmdsView;
            bool cmdsFound = false;
            try {
                // Get view of valid toml array
                cmdsView = util::toml::viewOfArray(transitionView, "cmd");
                cmdsFound = true;
            } catch (rstate::except::config_parse_error e) {
                RCLCPP_DEBUG(this->get_logger(), "No commands found in section '%s'", transitionName);
            }
            if (cmdsFound) {
                bool allowCancel;
                if (!strcmp(transitionName, "shutdown_unconfigred") || !strcmp(transitionName, "shutdown_inactive") ||
                    !strcmp(transitionName, "shutdown_active") || !strcmp(transitionName, "shutdown_armed")) {
                    allowCancel = false;
                };

                for (auto &&cmdNode : *cmdsView.as_array()) {
                    // Create command and add to execution vector of transition
                    auto cmd = this->createCmd(toml::node_view(cmdNode), allowCancel);
                    transitionCmdVector->push_back(cmd);
                }
            }
        }
    }

    std::shared_ptr<CmdIface> Node::createCmd(toml::node_view<toml::node> cmdView, bool allowCancel) {
        for (auto cmdType : cmdTypeMap) {
            auto cmdTypeName = cmdType.first;
            auto cmdTypeConstructor = cmdType.second;

            toml::node_view<toml::node> cmdTypeView;
            try {
                cmdTypeView = util::toml::viewOfTable(cmdView, cmdTypeName);
            } catch (rstate::except::config_parse_error e) {
                // Cmd was not of type cmdType
                // Catch and ignore
                // Exit current loop iteration
                continue;
            }

            auto cmd = cmdTypeConstructor(cmdTypeView, allowCancel);
            return cmd;
        }

        // If function did not return early, no match was found
        std::stringstream error;

        error << "Command found in .toml does not match any know type!\n";
        error << "TOML: " << cmdView << "\n";

        throw rstate::except::config_parse_error(error.str());
    }

    // Create cmd.service
    std::shared_ptr<CmdIface> Node::createCmdService(toml::node_view<toml::node> cmdServiceView, bool allowCancel) {
        std::string serviceType = util::toml::getTomlEntryByKey<std::string>(cmdServiceView, "service");

        // We must explicitly cast based on service type
        try {
            switch (cmdServiceTypeMap.at(serviceType)) {
            case 1: {
                return createCmdServiceType<lifecycle_msgs::srv::ChangeState>(
                    cmdServiceView, allowCancel, this->cmdServiceClientMapChangeState);
            } break;
            case 2: {
                return createCmdServiceType<lifecycle_msgs::srv::GetState>(
                    cmdServiceView, allowCancel, this->cmdServiceClientMapGetState);
            } break;
            default:
                throw std::out_of_range(
                    "ROS2 service type defined in 'cmdServiceTypeMap' but not in createCmdService() switch case");
            };
        } catch (const std::out_of_range &e) {
            std::stringstream error;

            error << "Service in toml section has no known type!\n";
            error << "Service: " << serviceType << "\n";
            error << "TOML: " << cmdServiceView << "\n";
            error << "Error: " << e.what() << "\n";

            throw rstate::except::config_parse_error(error.str());
        }
    }
} // namespace rstate

#include <node.hpp>

namespace rstate {
    // Node will construct and immediately enter the unconfigured state
    Node::Node() : rclcpp_lifecycle::LifecycleNode("rstate") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->setState(Unknown::getInstance());

        this->publisherNetworkTransitionEvent =
            this->create_publisher<rstate_msgs::msg::NetworkTransitionEvent>("rstate/network_transition_event", 10);
        this->publisherNetworkTransitionEvent->on_activate();

        this->serviceGetNetworkState = this->create_service<rstate_msgs::srv::GetNetworkState>(
            "rstate/get_network_state",
            std::bind(&Node::serviceGetNetworkStateCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->serviceGetAvailableNetworkTransistions =
            this->create_service<rstate_msgs::srv::GetAvailableNetworkTransitions>(
                "rstate/get_available_network_transitions",
                std::bind(&Node::serviceGetAvailableNetworkTransistionsCallback,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));

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

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    LifecycleCallbackReturn Node::on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            toml::table toml = toml::parse_file("/workspaces/rCTRL/ros2/src/rstate/config.toml");
            readConfig(toml);
        } catch (except::config_parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
            return LifecycleCallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        // Set initial network state
        this->setState(Unconfigured::getInstance());

        // Have to publish something to update gui, maybe consider properly extending the state machine?
        this->publishNetworkTransitionEvent(
            NetworkTransitionEnum::Configure, NetworkStateEnum::Unknown, NetworkStateEnum::Unconfigured);

        // Bind action server call backs to State interface method
        // Provide a reference to the State currently held by the rstate Node as an execution context
        this->actionServer = std::make_shared<ActionServer<rstate_msgs::srv::NetworkTransitionCancelGoal,
                                                           rstate_msgs::srv::NetworkTransitionSendGoal,
                                                           rstate_msgs::msg::NetworkTransitionFeedback>>(
            static_cast<rclcpp_lifecycle::LifecycleNode *>(this),
            "rstate/change_network_state",
            std::bind(&State::handleGoal, std::ref(this->currentState), this, std::placeholders::_1),
            std::bind(&State::handleCancel, std::ref(this->currentState), this, std::placeholders::_1),
            std::bind(&State::handleAccepted, std::ref(this->currentState), this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        this->actionServer.reset();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {
        this->actionServer.reset();

        this->cmdsOnConfigure.clear();
        this->cmdsOnCleanUp.clear();
        this->cmdsOnActivate.clear();
        this->cmdsOnDeactivate.clear();
        this->cmdsOnArm.clear();
        this->cmdsOnDisarm.clear();
        this->cmdsOnShutdownUnconfigured.clear();
        this->cmdOnShutdownInactive.clear();
        this->cmdsOnShutdownActive.clear();
        this->cmdsOnShutdownArmed.clear();
    }

    // Assign the state of the node and trigger enter() event
    void Node::setState(State &newState) {
        // this->currentState->exit(this); (this can be defined for all states and used if needed)
        this->currentState = &newState;
        this->currentState->enter(this);
    }

    void Node::publishNetworkTransitionEvent(NetworkTransitionEnum transition,
                                             NetworkStateEnum start_state,
                                             NetworkStateEnum goal_state) {
        rstate_msgs::msg::NetworkTransitionEvent transitionEvent;
        transitionEvent.timestamp = 0; // FIX
        transitionEvent.transition.id = (uint)transition;
        transitionEvent.transition.label = generateNetworkTransitionLabel(transition);
        transitionEvent.start_state.id = (uint)start_state;
        transitionEvent.start_state.label = generateNetworkStateLabel(start_state);
        transitionEvent.goal_state.id = (uint)goal_state;
        transitionEvent.goal_state.label = generateNetworkStateLabel(goal_state);

        this->publisherNetworkTransitionEvent->publish(transitionEvent);
    }

    void Node::serviceGetNetworkStateCallback(const std::shared_ptr<rstate_msgs::srv::GetNetworkState::Request> request,
                                              const std::shared_ptr<rstate_msgs::srv::GetNetworkState::Response> response) {
        (void)request;
        response->current_state = this->currentState->getNetworkState();
    }

    void Node::serviceGetAvailableNetworkTransistionsCallback(
        const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkTransitions::Request> request,
        const std::shared_ptr<rstate_msgs::srv::GetAvailableNetworkTransitions::Response> response) {
        (void)request;
        *response = this->currentState->getAvailableNetworkTransitions();
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
            } catch (rstate::except::config_parse_error &e) {
                RCLCPP_DEBUG(this->get_logger(), "No commands found in section '%s'", transitionName);
            }
            if (cmdsFound) {
                // Shutdown commands cannot be canceled
                // strcmp() returns 0 when strings match
                bool allowCancel = true;
                if ((strcmp(transitionName, "shutdown_unconfigured") == 0) ||
                    (strcmp(transitionName, "shutdown_inactive") == 0) ||
                    (strcmp(transitionName, "shutdown_active") == 0) || (strcmp(transitionName, "shutdown_armed") == 0)) {
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
            } catch (rstate::except::config_parse_error &e) {
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

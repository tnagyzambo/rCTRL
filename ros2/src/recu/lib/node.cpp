#include <node.hpp>
#include <ri2c_msgs/srv/detail/high_speed_data_logging_action__struct.hpp>

namespace recu {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("recu") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->declare_parameter<std::string>("config_path", "/home/ros/recu/config.toml");

        // Construct timers and immediately stop them
        this->ignitionSequenceTimer = this->create_wall_timer(5ms, std::bind(&Node::ignitionSequenceCallback, this));
        this->ignitionSequenceTimer->cancel();
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            toml::table toml = toml::parse_file(this->get_parameter("config_path").as_string());
            readConfig(toml);
        } catch (rgpio::except::config_parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        this->srvIgnitionSequence = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/fire", std::bind(&Node::IgnitionSequence, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/fire").c_str());

        this->srvBV_Open = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/bv/open", std::bind(&Node::BV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/open").c_str());

        this->srvBV_Close = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/bv/close", std::bind(&Node::BV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/close").c_str());

        this->srvBV_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/bv/get_state", std::bind(&Node::BV_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/get_state").c_str());

        this->srvPV_Open = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/pv/open", std::bind(&Node::PV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/open").c_str());

        this->srvPV_Close = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/pv/close", std::bind(&Node::PV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/close").c_str());

        this->srvPV_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/pv/get_state", std::bind(&Node::PV_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/get_state").c_str());

        this->srvMV1_Open = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/mv1/open", std::bind(&Node::MV1_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/open").c_str());

        this->srvMV1_Close = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/mv1/close", std::bind(&Node::MV1_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/close").c_str());

        this->srvMV1_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/mv1/get_state", std::bind(&Node::MV1_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/get_state").c_str());

        this->srvMV2_Open = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/mv2/open", std::bind(&Node::MV2_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/open").c_str());

        this->srvMV2_Close = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/mv2/close", std::bind(&Node::MV2_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/close").c_str());

        this->srvMV2_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/mv2/get_state", std::bind(&Node::MV2_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/get_state").c_str());

        this->clHighSpeedDataLoggingOn =
            this->create_client<ri2c_msgs::srv::HighSpeedDataLoggingAction>("ri2c/hs_datalog/on");

        this->clHighSpeedDataLoggingOff =
            this->create_client<ri2c_msgs::srv::HighSpeedDataLoggingAction>("ri2c/hs_datalog/off");

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        this->srvIgnitionSequence.reset();
        this->srvBV_Open.reset();
        this->srvBV_Close.reset();
        this->srvBV_GetState.reset();
        this->srvPV_Open.reset();
        this->srvPV_Close.reset();
        this->srvPV_GetState.reset();
        this->srvMV1_Open.reset();
        this->srvMV1_Close.reset();
        this->srvMV1_GetState.reset();
        this->srvMV2_Open.reset();
        this->srvMV2_Close.reset();
        this->srvMV2_GetState.reset();

        this->ignitionSequenceTimer->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->deleteAllPointers();

        this->ignitionSequenceEnd = -1ms;
        this->ignitionSequenceOpenBV = -1ms;
        this->ignitionSequenceCloseBV = -1ms;
        this->ignitionSequenceOpenPV = -1ms;
        this->ignitionSequenceClosePV = -1ms;
        this->ignitionSequenceOpenMV1 = -1ms;
        this->ignitionSequenceCloseMV1 = -1ms;
        this->ignitionSequenceOpenMV2 = -1ms;
        this->ignitionSequenceCloseMV2 = -1ms;
        this->ignitionSequenceOnIgnitor = -1ms;
        this->ignitionSequenceOffIgnitor = -1ms;
        this->ignitionSequenceOnHSDatalogging = -1ms;
        this->ignitionSequenceOffHSDatalogging = -1ms;

        this->ignitionSequenceTimer->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {
        this->valveBV.reset();
        this->valvePV.reset();
        this->valveMV1.reset();
        this->valveMV2.reset();
        this->pyro.reset();

        this->srvIgnitionSequence.reset();
        this->srvBV_Open.reset();
        this->srvBV_Close.reset();
        this->srvBV_GetState.reset();
        this->srvPV_Open.reset();
        this->srvPV_Close.reset();
        this->srvPV_GetState.reset();
        this->srvMV1_Open.reset();
        this->srvMV1_Close.reset();
        this->srvMV1_GetState.reset();
        this->srvMV2_Open.reset();
        this->srvMV2_Close.reset();
        this->srvMV2_GetState.reset();
    }

    // Parse .toml and create all commands
    void Node::readConfig(toml::table toml) {
        auto tomlView = toml::node_view(toml);

        auto gpioView = rutil::toml::viewOfTable(tomlView, "gpio");

        this->valveBV = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "valveBV"));
        this->valvePV = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "valvePV"));
        this->valveMV1 = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "valveMV1"));
        this->valveMV2 = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "valveMV2"));
        this->pyro = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "pyro"));

        auto ignitionSequenceView = rutil::toml::viewOfTable(tomlView, "ignition_sequence");

        auto ignitionSequenceBVView = rutil::toml::viewOfTable(ignitionSequenceView, "BV");
        this->ignitionSequenceOpenBV =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceBVView, "open");
        this->ignitionSequenceCloseBV =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceBVView, "close");

        auto ignitionSequencePVView = rutil::toml::viewOfTable(ignitionSequenceView, "PV");
        this->ignitionSequenceOpenPV =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequencePVView, "open");
        this->ignitionSequenceClosePV =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequencePVView, "close");

        auto ignitionSequenceMV1View = rutil::toml::viewOfTable(ignitionSequenceView, "MV1");
        this->ignitionSequenceOpenMV1 =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceMV1View, "open");
        this->ignitionSequenceCloseMV1 =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceMV1View, "close");

        auto ignitionSequenceMV2View = rutil::toml::viewOfTable(ignitionSequenceView, "MV2");
        this->ignitionSequenceOpenMV2 =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceMV2View, "open");
        this->ignitionSequenceCloseMV2 =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceMV2View, "close");

        auto ignitionSequenceIgnitorView = rutil::toml::viewOfTable(ignitionSequenceView, "ignitor");
        this->ignitionSequenceOnIgnitor =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceIgnitorView, "on");
        this->ignitionSequenceOffIgnitor =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceIgnitorView, "off");

        auto ignitionSequenceHSDataloggingView =
            rutil::toml::viewOfTable(ignitionSequenceView, "high_speed_data_logging");
        this->ignitionSequenceOnHSDatalogging =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceHSDataloggingView, "on");
        this->ignitionSequenceOffHSDatalogging =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceHSDataloggingView, "off");

        auto ignitionSequenceEndView = rutil::toml::viewOfTable(ignitionSequenceView, "end");
        this->ignitionSequenceEnd =
            (std::chrono::milliseconds)rutil::toml::getTomlEntryByKey<int>(ignitionSequenceEndView, "end");
    }

    void Node::ignitionSequenceCallback() {
        std::chrono::milliseconds ignitionSequenceTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - this->ignitionSequenceStartTime);

        if (ignitionSequenceTime >= this->ignitionSequenceOpenBV && this->ignitionSequenceOpenBV >= 0ms) {
            try {
                this->valveBV->write(rgpio::gpio::line_level::HIGH);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOpenBV = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceCloseBV && this->ignitionSequenceCloseBV >= 0ms) {
            try {
                this->valveBV->write(rgpio::gpio::line_level::LOW);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceCloseBV = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOpenPV && this->ignitionSequenceOpenPV >= 0ms) {
            try {
                this->valvePV->write(rgpio::gpio::line_level::HIGH);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOpenPV = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceClosePV && this->ignitionSequenceClosePV >= 0ms) {
            try {
                this->valvePV->write(rgpio::gpio::line_level::LOW);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceClosePV = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOpenMV1 && this->ignitionSequenceOpenMV1 >= 0ms) {
            try {
                this->valveMV1->write(rgpio::gpio::line_level::HIGH);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOpenMV1 = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceCloseMV1 && this->ignitionSequenceCloseMV1 >= 0ms) {
            try {
                this->valveMV1->write(rgpio::gpio::line_level::LOW);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceCloseMV1 = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOpenMV2 && this->ignitionSequenceOpenMV2 >= 0ms) {
            try {
                this->valveMV2->write(rgpio::gpio::line_level::HIGH);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOpenMV2 = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceCloseMV2 && this->ignitionSequenceCloseMV2 >= 0ms) {
            try {
                this->valveMV2->write(rgpio::gpio::line_level::LOW);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceCloseMV2 = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOnIgnitor && this->ignitionSequenceOnIgnitor >= 0ms) {
            try {
                this->pyro->write(rgpio::gpio::line_level::HIGH);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOnIgnitor = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOffIgnitor && this->ignitionSequenceOffIgnitor >= 0ms) {
            try {
                this->pyro->write(rgpio::gpio::line_level::LOW);
            } catch (rgpio::except::gpio_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->ignitionSequenceOffIgnitor = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOnHSDatalogging &&
            this->ignitionSequenceOnHSDatalogging >= 0ms) {
            try {
                auto request = std::make_shared<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request>();
                auto result = this->clHighSpeedDataLoggingOn->async_send_request(request);

                // transfer the future's shared state to a longer-lived future
                this->pending_futures.push_back(std::move(result));
            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }

            this->ignitionSequenceOnHSDatalogging = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceOffHSDatalogging &&
            this->ignitionSequenceOffHSDatalogging >= 0ms) {
            try {
                auto request = std::make_shared<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request>();
                auto result = this->clHighSpeedDataLoggingOff->async_send_request(request);

                // transfer the future's shared state to a longer-lived future
                this->pending_futures.push_back(std::move(result));
            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }

            this->ignitionSequenceOffHSDatalogging = -1ms;
        }

        if (ignitionSequenceTime >= this->ignitionSequenceEnd) {
            this->ignitionSequenceEnd = -1ms;
            this->ignitionSequenceTimer->cancel();
            RCLCPP_INFO(this->get_logger(), "Ignition sequence over");
        }
    }

    recu_msgs::srv::GetValveState::Response Node::createValveStateResponse(ValveState valveState) {
        recu_msgs::srv::GetValveState::Response response;

        response.current_state.id = valveState;

        switch (valveState) {
        case Closed:
            response.current_state.label = "closed";
            break;
        case Open:
            response.current_state.label = "open";
            break;
        case Unknown:
            response.current_state.label = "unknown";
            break;
        default:
            response.current_state.label = "unknown";
            break;
        }

        return response;
    }

    void Node::IgnitionSequence(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                                std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;

        RCLCPP_WARN(this->get_logger(), "FIRING!!!");
        this->ignitionSequenceStartTime = std::chrono::high_resolution_clock::now();
        this->ignitionSequenceTimer->reset();
    }

    void Node::BV_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveBV->write(rgpio::gpio::line_level::HIGH);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::BV_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveBV->write(rgpio::gpio::line_level::LOW);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::BV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                           std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        try {
            *response = createValveStateResponse((ValveState)rgpio::gpio::line_level::toInt(this->valveBV->getState()));
        } catch (rgpio::except::gpio_error &e) {
            *response = createValveStateResponse(ValveState::Unknown);
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::PV_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valvePV->write(rgpio::gpio::line_level::HIGH);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::PV_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valvePV->write(rgpio::gpio::line_level::LOW);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::PV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                           std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        try {
            *response = createValveStateResponse((ValveState)rgpio::gpio::line_level::toInt(this->valvePV->getState()));
        } catch (rgpio::except::gpio_error &e) {
            *response = createValveStateResponse(ValveState::Unknown);
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV1_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveMV1->write(rgpio::gpio::line_level::HIGH);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV1_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                         std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveMV1->write(rgpio::gpio::line_level::LOW);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV1_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                            std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        try {
            *response = createValveStateResponse((ValveState)rgpio::gpio::line_level::toInt(this->valveMV1->getState()));
        } catch (rgpio::except::gpio_error &e) {
            *response = createValveStateResponse(ValveState::Unknown);
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV2_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveMV2->write(rgpio::gpio::line_level::HIGH);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV2_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                         std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        try {
            this->valveMV2->write(rgpio::gpio::line_level::LOW);
        } catch (rgpio::except::gpio_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::MV2_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                            std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        try {
            *response = createValveStateResponse((ValveState)rgpio::gpio::line_level::toInt(this->valveMV2->getState()));
        } catch (rgpio::except::gpio_error &e) {
            *response = createValveStateResponse(ValveState::Unknown);
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }
} // namespace recu

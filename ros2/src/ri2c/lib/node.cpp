#include <node.hpp>

namespace ri2c {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("ri2c") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->declare_parameter<std::string>("config_path", "/home/ros/ri2c/config.toml");
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            toml::table toml = toml::parse_file(this->get_parameter("config_path").as_string());
            readConfig(toml);
        } catch (ri2c::except::config_parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Get handle for i2c bus
        this->i2cBus = open(this->i2cBusName.c_str(), O_RDWR);
        if (this->i2cBus < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open i2c bus '%s'", this->i2cBusName.c_str());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        this->p_h2o2->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->p_ethanol->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->p_pressurant->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->loadcell->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->p_chamber->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->p_manifold->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->t_chamber->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));

        try {
            this->loggerLowSpeed = std::make_unique<rdata::Logger>("/home/ros/rdata/influx/credentials.toml");
            this->loggerHighSpeed = std::make_unique<rdata::Logger>("/home/ros/rdata/influx/credentials.toml");
        } catch (const std::runtime_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node!\nWhat: %s", e.what());

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Construct timers and immediately stop them
        this->timerDataLoggingLowSpeed =
            this->create_wall_timer(samplePeriodLowSpeed, std::bind(&Node::callbackDataLoggingLowSpeed, this));
        this->timerDataLoggingLowSpeed->cancel();

        this->timerDataLoggingLowSpeedWrite =
            this->create_wall_timer(loggingPeriodLowSpeed, std::bind(&Node::callbackDataLoggingLowSpeedWrite, this));
        this->timerDataLoggingLowSpeedWrite->cancel();

        this->timerDataLoggingHighSpeed =
            this->create_wall_timer(samplePeriodHighSpeed, std::bind(&Node::callbackDataLoggingHighSpeed, this));
        this->timerDataLoggingHighSpeed->cancel();

        this->timerPressureControlLoop =
            this->create_wall_timer(pressureControlLoopRate, std::bind(&Node::callbackPressureControlLoop, this));
        this->timerPressureControlLoop->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        // Start timers
        this->timerDataLoggingLowSpeed->reset();
        this->timerDataLoggingLowSpeedWrite->reset();

        this->srvDataLoggingHighSpeedOn = this->create_service<ri2c_msgs::srv::HighSpeedDataLoggingAction>(
            "ri2c/hs_datalog/on",
            std::bind(&Node::callbackDataLoggingHighSpeedOn, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("ri2c/hs_datalog/on").c_str());

        this->srvDataLoggingHighSpeedOff = this->create_service<ri2c_msgs::srv::HighSpeedDataLoggingAction>(
            "ri2c/hs_datalog/off",
            std::bind(&Node::callbackDataLoggingHighSpeedOff, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("ri2c/hs_datalog/off").c_str());

        this->srvPresControlOn = this->create_service<ri2c_msgs::srv::PresControlLoopAction>(
            "ri2c/p_control/open",
            std::bind(&Node::callbackPresControlOn, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("ri2c/p_control/open").c_str());

        this->srvPresControlOff = this->create_service<ri2c_msgs::srv::PresControlLoopAction>(
            "ri2c/p_control/close",
            std::bind(&Node::callbackPresControlOff, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("ri2c/p_control/close").c_str());

        this->presControlState = false;
        this->srvPresControlGetState = this->create_service<ri2c_msgs::srv::GetValveState>(
            "ri2c/p_control/get_state",
            std::bind(&Node::PresControl_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("ri2c/p_control/get_state").c_str());

        this->clPV_Open = this->create_client<recu_msgs::srv::ValveAction>("recu/pv/open");
        this->clPV_Close = this->create_client<recu_msgs::srv::ValveAction>("recu/pv/close");

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        // Stop timers
        this->timerDataLoggingLowSpeed->cancel();
        this->timerDataLoggingHighSpeed->cancel();
        this->timerPressureControlLoop->cancel();

        // Close PV Valve
        try {
            auto request = std::make_shared<recu_msgs::srv::ValveAction::Request>();
            auto result = this->clPV_Close->async_send_request(request);

        } catch (std::runtime_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        close(this->i2cBus);

        this->deleteAllPointers();

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
        this->p_h2o2.reset();
        this->p_ethanol.reset();
        this->p_pressurant.reset();
        this->loadcell.reset();
        this->p_chamber.reset();
        this->p_manifold.reset();
        this->t_chamber.reset();

        this->timerDataLoggingLowSpeed.reset();
        this->timerDataLoggingLowSpeedWrite.reset();
        this->timerDataLoggingHighSpeed.reset();
        this->timerPressureControlLoop.reset();

        this->loggerLowSpeed.reset();
        this->loggerHighSpeed.reset();

        this->srvDataLoggingHighSpeedOn.reset();
        this->srvDataLoggingHighSpeedOff.reset();

        this->srvPresControlOn.reset();
        this->srvPresControlOff.reset();
        this->srvPresControlGetState.reset();
    }

    void Node::readConfig(toml::table toml) {
        try {
            auto tomlView = toml::node_view(toml);

            auto i2cView = rutil::toml::viewOfTable(tomlView, "i2c");

            this->i2cBusName = rutil::toml::getTomlEntryByKey<std::string>(i2cView, "bus");

            auto ads1014View = rutil::toml::viewOfTable(i2cView, "ads1014");

            this->p_h2o2 =
                std::make_unique<PAA_7LC_30BAR>(PAA_7LC_30BAR(rutil::toml::viewOfTable(ads1014View, "p_h2o2")));
            this->p_ethanol =
                std::make_unique<PAA_7LC_30BAR>(PAA_7LC_30BAR(rutil::toml::viewOfTable(ads1014View, "p_ethanol")));
            this->p_pressurant = std::make_unique<PAA_7LHPC_400BAR>(
                PAA_7LHPC_400BAR(rutil::toml::viewOfTable(ads1014View, "p_pressurant")));
            this->loadcell =
                std::make_unique<LoadcellBridge>(LoadcellBridge(rutil::toml::viewOfTable(ads1014View, "loadcell")));
            this->p_chamber =
                std::make_unique<M5HB_30BAR>(M5HB_30BAR(rutil::toml::viewOfTable(ads1014View, "p_chamber")));
            this->p_manifold =
                std::make_unique<M5HB_30BAR>(M5HB_30BAR(rutil::toml::viewOfTable(ads1014View, "p_manifold")));
            this->t_chamber = std::make_unique<K_TYPE>(K_TYPE(rutil::toml::viewOfTable(ads1014View, "t_chamber")));

            auto loggingView = rutil::toml::viewOfTable(tomlView, "data_logging");

            auto lowSpeedView = rutil::toml::viewOfTable(loggingView, "low_speed");

            this->samplePeriodLowSpeed =
                std::chrono::milliseconds(rutil::toml::getTomlEntryByKey<int>(lowSpeedView, "sample_period"));

            this->loggingPeriodLowSpeed =
                std::chrono::milliseconds(rutil::toml::getTomlEntryByKey<int>(lowSpeedView, "logging_period"));

            auto highSpeedView = rutil::toml::viewOfTable(loggingView, "high_speed");

            this->samplePeriodHighSpeed =
                std::chrono::milliseconds(rutil::toml::getTomlEntryByKey<int>(highSpeedView, "sample_period"));

            // Read pressure control configuration settings
            auto pressureControlView = rutil::toml::viewOfTable(tomlView, "pressureControl");

            this->pressureControlLoopRate = std::chrono::milliseconds(
                rutil::toml::getTomlEntryByKey<int>(pressureControlView, "pressureControlPeriod"));
            this->pressureSetPoint = (float)rutil::toml::getTomlEntryByKey<int>(pressureControlView, "pressureSetPoint");
            this->lastPressureControlCommand = false;

        } catch (rutil::except::toml_parse_error &e) {
            throw ri2c::except::config_parse_error(e.what());
        }
    }

    // Write to buffer
    void Node::callbackDataLoggingLowSpeed() {
        float value1 = this->p_h2o2->read(this->i2cBus);
        float value2 = this->p_ethanol->read(this->i2cBus);
        float value3 = this->p_pressurant->read(this->i2cBus);
        float value4 = this->loadcell->read(this->i2cBus);
        float value5 = this->p_chamber->read(this->i2cBus);
        float value6 = this->p_manifold->read(this->i2cBus);
        float value7 = this->t_chamber->read(this->i2cBus);

        this->loggerLowSpeed->log(fmt::format("sensor=p_h2o2 Bar={}", value1));
        this->loggerLowSpeed->log(fmt::format("sensor=p_ethanol Bar={}", value2));
        this->loggerLowSpeed->log(fmt::format("sensor=p_pressurant Bar={}", value3));
        this->loggerLowSpeed->log(fmt::format("sensor=loadcell Volts={}", value4));
        this->loggerLowSpeed->log(fmt::format("sensor=p_chamber Bar={}", value5));
        this->loggerLowSpeed->log(fmt::format("sensor=p_manifold Bar={}", value6));
        this->loggerLowSpeed->log(fmt::format("sensor=t_chamber Volts={}", value7));
    }

    // Write buffer to influx
    void Node::callbackDataLoggingLowSpeedWrite() { this->loggerLowSpeed->writeToInflux(); }

    void Node::callbackDataLoggingHighSpeed() {
        float value1 = this->p_h2o2->read(this->i2cBus);
        float value2 = this->p_ethanol->read(this->i2cBus);
        float value3 = this->p_pressurant->read(this->i2cBus);
        float value4 = this->loadcell->read(this->i2cBus);
        float value5 = this->p_chamber->read(this->i2cBus);
        float value6 = this->p_manifold->read(this->i2cBus);
        float value7 = this->t_chamber->read(this->i2cBus);

        this->loggerLowSpeed->log(fmt::format("sensor=p_h2o2 Bar={}", value1));
        this->loggerLowSpeed->log(fmt::format("sensor=p_ethanol Bar={}", value2));
        this->loggerLowSpeed->log(fmt::format("sensor=p_pressurant Bar={}", value3));
        this->loggerLowSpeed->log(fmt::format("sensor=loadcell Volts={}", value4));
        this->loggerLowSpeed->log(fmt::format("sensor=p_chamber Bar={}", value5));
        this->loggerLowSpeed->log(fmt::format("sensor=p_manifold Bar={}", value6));
        this->loggerLowSpeed->log(fmt::format("sensor=t_chamber Volts={}", value7));
    }

    void Node::callbackDataLoggingHighSpeedOn(
        const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request> request,
        std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response> response) {
        (void)request;
        (void)response;
        this->timerDataLoggingHighSpeed->reset();
        RCLCPP_INFO(this->get_logger(), "High Speed Data Logging On");
    }

    void Node::callbackDataLoggingHighSpeedOff(
        const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request> request,
        std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response> response) {
        (void)request;
        (void)response;
        this->timerDataLoggingHighSpeed->cancel();
        this->loggerHighSpeed->writeToInflux();
        RCLCPP_INFO(this->get_logger(), "High Speed Data Logging Off");
    }

    void Node::callbackPresControlOn(const std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Request> request,
                                     std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Response> response) {
        (void)request;
        (void)response;
        // Turn on control loop timer
        this->timerPressureControlLoop->reset();
        this->presControlState = true;
        this->lastPressureControlCommand = false;
    }

    void Node::callbackPresControlOff(const std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Request> request,
                                      std::shared_ptr<ri2c_msgs::srv::PresControlLoopAction::Response> response) {
        (void)request;
        (void)response;
        // Turn off control loop timer and close valve
        this->presControlState = false;
        this->lastPressureControlCommand = false;
        this->timerPressureControlLoop->cancel();
        // Close PV Valve
        try {
            auto request = std::make_shared<recu_msgs::srv::ValveAction::Request>();
            auto result = this->clPV_Close->async_send_request(request);

        } catch (std::runtime_error &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void Node::PresControl_GetState(const std::shared_ptr<ri2c_msgs::srv::GetValveState::Request> request,
                                    std::shared_ptr<ri2c_msgs::srv::GetValveState::Response> response) {
        (void)request;

        *response = createValveStateResponse(this->presControlState);
    }

    void Node::callbackPressureControlLoop() {
        float pressureTank = this->p_h2o2->read(this->i2cBus);
        float pressureE = this->pressureSetPoint - pressureTank; // Create error signal

        // If pressure is lower than set point
        // AND the last command sent was to turn the valve off (false)
        if (pressureE > 0 && !this->lastPressureControlCommand) {
            // Request to open PV
            try {
                auto request = std::make_shared<recu_msgs::srv::ValveAction::Request>();
                auto result = this->clPV_Open->async_send_request(request);

            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->lastPressureControlCommand = true;
            RCLCPP_INFO(this->get_logger(), "Powered pressurization valve. PE: %f", pressureE);

        } else if (pressureE < 0 && this->lastPressureControlCommand) {
            // Request to close PV
            try {
                auto request = std::make_shared<recu_msgs::srv::ValveAction::Request>();
                auto result = this->clPV_Close->async_send_request(request);

            } catch (std::runtime_error &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
            this->lastPressureControlCommand = false;
            RCLCPP_INFO(this->get_logger(), "Unpowered pressurization valve. PE: %f", pressureE);
        }
    }

    ri2c_msgs::srv::GetValveState::Response Node::createValveStateResponse(bool valveState) {
        ri2c_msgs::srv::GetValveState::Response response;

        response.current_state.id = (int)valveState;

        switch ((int)valveState) {
        case true:
            response.current_state.label = "on";
            break;
        case false:
            response.current_state.label = "off";
            break;
        }
        return response;
    }
} // namespace ri2c

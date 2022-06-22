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
        this->loadcell->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->p_chamber->init(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        this->t_chamber->init(this->i2cBus);

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
        this->srvDataLoggingHighSpeedOff = this->create_service<ri2c_msgs::srv::HighSpeedDataLoggingAction>(
            "ri2c/hs_datalog/off",
            std::bind(&Node::callbackDataLoggingHighSpeedOff, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        // Stop timers
        this->timerDataLoggingLowSpeed->cancel();
        this->timerDataLoggingHighSpeed->cancel();

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
        this->loadcell.reset();
        this->p_chamber.reset();
        this->t_chamber.reset();

        this->timerDataLoggingLowSpeed.reset();
        this->timerDataLoggingLowSpeedWrite.reset();
        this->timerDataLoggingHighSpeed.reset();

        this->loggerLowSpeed.reset();
        this->loggerHighSpeed.reset();

        this->srvDataLoggingHighSpeedOn.reset();
        this->srvDataLoggingHighSpeedOff.reset();
    }

    void Node::readConfig(toml::table toml) {
        try {
            auto tomlView = toml::node_view(toml);

            auto i2cView = rutil::toml::viewOfTable(tomlView, "i2c");

            this->i2cBusName = rutil::toml::getTomlEntryByKey<std::string>(i2cView, "bus");

            auto ads1014View = rutil::toml::viewOfTable(i2cView, "ads1014");

            this->p_h2o2 = std::make_unique<PAA_7LC_30BAR>(PAA_7LC_30BAR(rutil::toml::viewOfTable(ads1014View, "p_h2o2")));
            this->loadcell = std::make_unique<LoadcellBridge>(LoadcellBridge(rutil::toml::viewOfTable(ads1014View, "loadcell")));
            this->p_chamber = std::make_unique<M5HB_30BAR>(M5HB_30BAR(rutil::toml::viewOfTable(ads1014View, "p_chamber")));
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
        } catch (rutil::except::toml_parse_error &e) {
            throw ri2c::except::config_parse_error(e.what());
        }
    }

    // Write to buffer
    void Node::callbackDataLoggingLowSpeed() {
        float value1 = this->p_h2o2->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value2 = this->loadcell->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value3 = this->p_chamber->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value4 = this->t_chamber->read(this->i2cBus);

        this->loggerLowSpeed->log(fmt::format("sensor=p_h2o2 Bar={}", value1));
        this->loggerLowSpeed->log(fmt::format("sensor=loadcell Volts={}", value2));
        this->loggerLowSpeed->log(fmt::format("sensor=p_chamber Bar={}", value3));
        this->loggerLowSpeed->log(fmt::format("sensor=t_chamber Volts={}", value4));
    }

    // Write buffer to influx
    void Node::callbackDataLoggingLowSpeedWrite() { this->loggerLowSpeed->writeToInflux(); }

    void Node::callbackDataLoggingHighSpeed() {
        float value1 = this->p_h2o2->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value2 = this->loadcell->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value3 = this->p_chamber->read(this->i2cBus);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
        float value4 = this->t_chamber->read(this->i2cBus);

        this->loggerHighSpeed->log(fmt::format("sensor=p_h2o2 Bar={}", value1));
        this->loggerHighSpeed->log(fmt::format("sensor=loadcell Volts={}", value2));
        this->loggerHighSpeed->log(fmt::format("sensor=p_chamber Bar={}", value3));
        this->loggerHighSpeed->log(fmt::format("sensor=t_chamber Volts={}", value4));
    }

    void Node::callbackDataLoggingHighSpeedOn(
        const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request> request,
        std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response> response) {
        (void)request;
        (void)response;
        this->timerDataLoggingHighSpeed->reset();
    }

    void Node::callbackDataLoggingHighSpeedOff(
        const std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Request> request,
        std::shared_ptr<ri2c_msgs::srv::HighSpeedDataLoggingAction::Response> response) {
        (void)request;
        (void)response;
        this->timerDataLoggingHighSpeed->cancel();
    }
} // namespace ri2c

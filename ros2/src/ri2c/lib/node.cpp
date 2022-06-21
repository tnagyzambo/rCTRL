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

        this->sensor1->init(this->i2cBus);
        // this->sensor2->init(this->i2cBus);
        // this->sensor3->init(this->i2cBus);
        // this->sensor4->init(this->i2cBus);

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
        this->sensor1.reset();
        // this->sensor2.reset();
        // this->sensor3.reset();
        // this->sensor4.reset();

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

            this->sensor1 = std::make_unique<ADS1014>(ADS1014(rutil::toml::viewOfTable(ads1014View, "sensor1")));
            // this->sensor2 = std::make_unique<ADS1014>(ADS1014(rutil::toml::viewOfTable(ads1014View, "sensor2")));
            // this->sensor3 = std::make_unique<ADS1014>(ADS1014(rutil::toml::viewOfTable(ads1014View, "sensor3")));
            // this->sensor4 = std::make_unique<ADS1014>(ADS1014(rutil::toml::viewOfTable(ads1014View, "sensor4")));

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
        float value1 = this->sensor1->read(this->i2cBus);
        // float value2 = this->sensor2->read(this->i2cBus);
        // float value3 = this->sensor3->read(this->i2cBus);
        // float value4 = this->sensor4->read(this->i2cBus);

        this->loggerLowSpeed->log(fmt::format("sensor=sensor1 value={}", value1));
        // this->loggerLowSpeed->log(fmt::format("sensor=sensor1 value={}, sensor=sensor2 value={}, sensor=sensor3
        // value={}, sensor=sensor3 value={}", value1, value2, value3, value4));
    }

    // Write buffer to influx
    void Node::callbackDataLoggingLowSpeedWrite() { this->loggerLowSpeed->writeToInflux(); }

    void Node::callbackDataLoggingHighSpeed() {
        float value1 = this->sensor1->read(this->i2cBus);
        // float value2 = this->sensor2->read(this->i2cBus);
        // float value3 = this->sensor3->read(this->i2cBus);
        // float value4 = this->sensor4->read(this->i2cBus);

        this->loggerHighSpeed->log(fmt::format("sensor=sensor1 value={}", value1));
        // this->loggerHighSpeed->log(fmt::format("sensor=sensor1 value={}, sensor=sensor2 value={}, sensor=sensor3
        // value={}, sensor=sensor3 value={}", value1, value2, value3, value4));
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

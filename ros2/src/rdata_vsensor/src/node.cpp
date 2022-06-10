#include <node.hpp>

namespace rdata::vsensor {
    Node::Node(const char *nodeName, std::chrono::milliseconds samplePeriod, std::chrono::milliseconds loggingPeriod)
        : rclcpp_lifecycle::LifecycleNode(nodeName) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->nodeName = nodeName;
        this->samplePeriod = samplePeriod;

        // Construct timers and immediately stop them
        this->sampleTimer = this->create_wall_timer(samplePeriod, std::bind(&Node::sampleCallback, this));
        this->sampleTimer->cancel();

        this->loggerTimer = this->create_wall_timer(loggingPeriod, std::bind(&Node::loggerCallback, this));
        this->loggerTimer->cancel();
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    // Callback to execute upon receiving configure command
    // If this returns SUCCESS the node will enter the inactive state
    // If this returns FAILURE the node will return to the unconfigured state
    // If this returns ERROR the node run the errorProcessing callback
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            this->logger = std::make_unique<rdata::Logger>("/home/ros/rdata/influx/credentials.toml");
        } catch (const std::runtime_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node\nWhat: %s", e.what());

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Callback to execute upon receiving activate command
    // If this returns SUCCESS the node will enter the active state
    // If this returns FAILURE the node will return to the inactive state
    // If this returns ERROR the node run the errorProcessing callback
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        // Start timers
        this->sampleTimer->reset();
        this->loggerTimer->reset();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Callback to execute upon receiving deactivate command
    // If this returns SUCCESS the node will enter the inactive state
    // FAILURE is not a valid return during this transition
    // If this returns ERROR the node run the errorProcessing callback
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        // Stop timers
        this->sampleTimer->cancel();
        this->loggerTimer->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Callback to execute upon receiving cleanup command
    // If this returns SUCCESS the node will enter the unconfigured state
    // FAILURE is not a valid return during this transition
    // If this returns ERROR the node run the errorProcessing callback
    // Drop all smart pointers and reset influx client
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->sampleTimer.reset();
        this->loggerTimer->reset();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Callback to execute upon receiving cleanup command
    // If this returns SUCCESS the node will enter the finalized state
    // FAILURE is not a valid return during this transition
    // If this returns ERROR the node run the errorProcessing callback
    // Drop all smart pointers
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        // Stop times
        this->sampleTimer->cancel();
        this->loggerTimer->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    uint Node::calcElapsedTime() {
        auto now = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->startTime).count();

        return ms;
    }

    void Node::sampleCallback() {}

    // Periodically write logs to influxdb
    // Optimal data rate is ~5000 lines of line protocol per write
    void Node::loggerCallback() { this->logger->writeToInflux(); }

    // Virtual boolean sensor
    Bool::Bool(const char *nodeName, std::chrono::milliseconds samplePeriod, std::chrono::milliseconds loggingPeriod)
        : Node(nodeName, samplePeriod, loggingPeriod) {

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual bool sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    Bool::~Bool() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual bool sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    void Bool::sampleCallback() {
        this->logger->log(fmt::format("sensor={} vBool={}", "virtual_bool", !this->prevOutput));
        this->prevOutput = !this->prevOutput;
    }

    // Virtual float sensor
    F64::F64(const char *nodeName,
             std::chrono::milliseconds samplePeriod,
             std::chrono::milliseconds loggingPeriod,
             std::chrono::milliseconds sinPeriod)
        : Node(nodeName, samplePeriod, loggingPeriod) {
        this->sinPeriod = 2 * 3.14 / (100 * sinPeriod.count());

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual f64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    F64::~F64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual f64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    void F64::sampleCallback() {
        uint ms = this->calcElapsedTime();
        this->logger->log(fmt::format("sensor={} vF64={}", "virtual_float", sin(this->sinPeriod * ms)));
    }

    // Virtual int sensor
    I64::I64(const char *nodeName, std::chrono::milliseconds samplePeriod, std::chrono::milliseconds loggingPeriod)
        : Node(nodeName, samplePeriod, loggingPeriod) {

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual i64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    I64::~I64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual i64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    void I64::sampleCallback() { this->logger->log(fmt::format("sensor={} vI64={}i", "virtual_int", rand() % 10)); }

    // Virtual string sensor
    Str::Str(const char *nodeName, std::chrono::milliseconds samplePeriod, std::chrono::milliseconds loggingPeriod)
        : Node(nodeName, samplePeriod, loggingPeriod) {

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual string sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    Str::~Str() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual string sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    void Str::sampleCallback() {
        uint ms = this->calcElapsedTime();
        this->logger->log(fmt::format("sensor={} vStr=\"Event at {}ms\"", "virtual_string", ms));
    }

    // Virtual uint sensor
    U64::U64(const char *nodeName, std::chrono::milliseconds samplePeriod, std::chrono::milliseconds loggingPeriod)
        : Node(nodeName, samplePeriod, loggingPeriod) {

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual u64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    U64::~U64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual u64 sensor with sample period '%sms'",
                    std::to_string(this->samplePeriod.count()).c_str());
    }

    void U64::sampleCallback() {
        this->logger->log(fmt::format("sensor={} vU64={}u", "virtual_uint", rand() % 10 + 4000000000));
    }
} // namespace rdata::vsensor

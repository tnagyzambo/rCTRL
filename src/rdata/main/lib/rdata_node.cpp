#include <rdata_node.hpp>

// Node will construct and immediately enter the unconfigured state
rdata::Node::Node() : rclcpp_lifecycle::LifecycleNode(iface::nodeName.data())
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::constructing);

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::unconfigured);
}

rdata::Node::~Node()
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::destructing);
}

// Callback to execute upon receiving configure command
// If this returns SUCCESS the node will enter the inactive state
// If this returns FAILURE the node will return to the unconfigured state
// If this returns ERROR the node run the errorProcessing callback
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::configuring);

    // Maintain a unique pointer to the influx client class
    // This lets us delay initialization of the influx client to the configure transition
    // It also lets us easily delete the influx client during the clean up transistion to return to the unconfigured state
    // This call will block until user input
    this->influxClient = std::make_unique<influx::Client>();

    // Bool
    this->srvCreateLoggerBool = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_bool.data(),
                                                                               std::bind(&rdata::Node::createLoggerBool,
                                                                                         this,
                                                                                         std::placeholders::_1,
                                                                                         std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_create_logger_bool>);

    // F64
    this->srvCreateLoggerF64 = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_f64.data(),
                                                                              std::bind(&rdata::Node::createLoggerF64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_create_logger_f64>);

    // I64
    this->srvCreateLoggerI64 = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_i64.data(),
                                                                              std::bind(&rdata::Node::createLoggerI64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_create_logger_i64>);

    // Str
    this->srvCreateLoggerStr = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_str.data(),
                                                                              std::bind(&rdata::Node::createLoggerStr,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_create_logger_str>);

    // U64
    this->srvCreateLoggerU64 = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_u64.data(),
                                                                              std::bind(&rdata::Node::createLoggerU64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_create_logger_u64>);

    // Bool
    this->srvRemoveLoggerBool = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_bool.data(),
                                                                               std::bind(&rdata::Node::removeLoggerBool,
                                                                                         this,
                                                                                         std::placeholders::_1,
                                                                                         std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_remove_logger_bool>);

    // F64
    this->srvRemoveLoggerF64 = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_f64.data(),
                                                                              std::bind(&rdata::Node::removeLoggerF64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_remove_logger_f64>);

    // I64
    this->srvRemoveLoggerI64 = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_i64.data(),
                                                                              std::bind(&rdata::Node::removeLoggerI64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_remove_logger_i64>);

    // Str
    this->srvRemoveLoggerStr = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_str.data(),
                                                                              std::bind(&rdata::Node::removeLoggerStr,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_remove_logger_str>);

    // U64
    this->srvRemoveLoggerF64 = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_u64.data(),
                                                                              std::bind(&rdata::Node::removeLoggerU64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::srv::created<iface::srv_remove_logger_u64>);

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::inactive);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving activate command
// If this returns SUCCESS the node will enter the active state
// If this returns FAILURE the node will return to the inactive state
// If this returns ERROR the node run the errorProcessing callback
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::activating);
    // Cannot activate subscribers
    // REFERENCE: https://github.com/ros2/demos/issues/488

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::active);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving deactivate command
// If this returns SUCCESS the node will enter the inactive state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::deactivating);
    // Cannot deactivate subscribers
    // REFERENCE: https://github.com/ros2/demos/issues/488

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::inactive);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving cleanup command
// If this returns SUCCESS the node will enter the unconfigured state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
// Drop all smart pointers and reset influx client
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::cleaningUp);

    this->deleteAllPointers();

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::unconfigured);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving cleanup command
// If this returns SUCCESS the node will enter the finalized state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
// Drop all smart pointers
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::shuttingDown);

    this->deleteAllPointers();

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::finalized);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void rdata::Node::deleteAllPointers()
{
    this->influxClient.reset();

    this->srvCreateLoggerF64.reset();

    this->srvRemoveLoggerF64.reset();

    this->loggersBool.clear();
    this->loggersF64.clear();
    this->loggersI64.clear();
    this->loggersStr.clear();
    this->loggersU64.clear();
}

// Service functions to create a new data logger of a given type
// This logic should be seperated across dedicated services on a per type basis since all requestors
// will know the type of datalogger that they will be requisting at compile time (avoid last minute decision making)
// These functions will append the created subscriber to the approriate vector
// These functions cannot be templated as it would require templated function pointers in the std::bind of the callback function
void rdata::Node::createLoggerBool(const std::shared_ptr<rdata::srv::CreateLogger::Request> request,
                                   std::shared_ptr<rdata::srv::CreateLogger::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogBool>(request->topic.c_str(),
                                                                 10,
                                                                 std::bind(&rdata::Node::callbackLogBool, this, std::placeholders::_1),
                                                                 opts);

    rdata::Logger<rdata::msg::LogBool> logger = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->loggersBool.push_back(logger);

    const char *topic = this->loggersBool.back().subPtr->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::created(topic).c_str());

    response->completed = true;
}

void rdata::Node::createLoggerF64(const std::shared_ptr<rdata::srv::CreateLogger::Request> request,
                                  std::shared_ptr<rdata::srv::CreateLogger::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogF64>(request->topic.c_str(),
                                                                10,
                                                                std::bind(&rdata::Node::callbackLogF64, this, std::placeholders::_1),
                                                                opts);

    rdata::Logger<rdata::msg::LogF64> logger = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->loggersF64.push_back(logger);

    const char *topic = this->loggersF64.back().subPtr->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::created(topic).c_str());

    response->completed = true;
}

void rdata::Node::createLoggerI64(const std::shared_ptr<rdata::srv::CreateLogger::Request> request,
                                  std::shared_ptr<rdata::srv::CreateLogger::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogI64>(request->topic.c_str(),
                                                                10,
                                                                std::bind(&rdata::Node::callbackLogI64, this, std::placeholders::_1),
                                                                opts);

    rdata::Logger<rdata::msg::LogI64> logger = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->loggersI64.push_back(logger);

    const char *topic = this->loggersI64.back().subPtr->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::created(topic).c_str());

    response->completed = true;
}

void rdata::Node::createLoggerStr(const std::shared_ptr<rdata::srv::CreateLogger::Request> request,
                                  std::shared_ptr<rdata::srv::CreateLogger::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogStr>(request->topic.c_str(),
                                                                10,
                                                                std::bind(&rdata::Node::callbackLogStr, this, std::placeholders::_1),
                                                                opts);

    rdata::Logger<rdata::msg::LogStr> logger = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->loggersStr.push_back(logger);

    const char *topic = this->loggersStr.back().subPtr->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::created(topic).c_str());

    response->completed = true;
}

void rdata::Node::createLoggerU64(const std::shared_ptr<rdata::srv::CreateLogger::Request> request,
                                  std::shared_ptr<rdata::srv::CreateLogger::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogU64>(request->topic.c_str(),
                                                                10,
                                                                std::bind(&rdata::Node::callbackLogU64, this, std::placeholders::_1),
                                                                opts);

    rdata::Logger<rdata::msg::LogU64> logger = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->loggersU64.push_back(logger);

    const char *topic = this->loggersU64.back().subPtr->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::created(topic).c_str());

    response->completed = true;
}

// Service functions to remove data loggers by topic name
void rdata::Node::removeLoggerBool(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                   std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersBool, request->topic.c_str());
    int loggersRemoved = this->loggersBool.size() - loggersTranformed.size();
    this->loggersBool = loggersTranformed;

    // Log the number of subscriptions removed
    for (int i = 0; i < loggersRemoved; i++)
    {
        const char *topic = request->topic.c_str();
        RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::removed(topic).c_str());
    };

    response->completed = true;
}

void rdata::Node::removeLoggerF64(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                  std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersF64, request->topic.c_str());
    int loggersRemoved = this->loggersF64.size() - loggersTranformed.size();
    this->loggersF64 = loggersTranformed;

    // Log the number of subscriptions removed
    for (int i = 0; i < loggersRemoved; i++)
    {
        const char *topic = request->topic.c_str();
        RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::removed(topic).c_str());
    };

    response->completed = true;
}

void rdata::Node::removeLoggerI64(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                  std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersI64, request->topic.c_str());
    int loggersRemoved = this->loggersI64.size() - loggersTranformed.size();
    this->loggersI64 = loggersTranformed;

    // Log the number of subscriptions removed
    for (int i = 0; i < loggersRemoved; i++)
    {
        const char *topic = request->topic.c_str();
        RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::removed(topic).c_str());
    };

    response->completed = true;
}

void rdata::Node::removeLoggerStr(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                  std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersStr, request->topic.c_str());
    int loggersRemoved = this->loggersStr.size() - loggersTranformed.size();
    this->loggersStr = loggersTranformed;

    // Log the number of subscriptions removed
    for (int i = 0; i < loggersRemoved; i++)
    {
        const char *topic = request->topic.c_str();
        RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::removed(topic).c_str());
    };

    response->completed = true;
}

void rdata::Node::removeLoggerU64(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                  std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersU64, request->topic.c_str());
    int loggersRemoved = this->loggersU64.size() - loggersTranformed.size();
    this->loggersU64 = loggersTranformed;

    // Log the number of subscriptions removed
    for (int i = 0; i < loggersRemoved; i++)
    {
        const char *topic = request->topic.c_str();
        RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::sub::rt::removed(topic).c_str());
    };

    response->completed = true;
}

// The callbacks needed by the ros2 subscriptions cannot be templated so this must be done through overloading
// The function call 'tryToWriteToInfluxDB()' can be templated however which lets us define uniform error handling logic once for all overloads
void rdata::Node::callbackLogBool(const rdata::msg::LogBool::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void rdata::Node::callbackLogF64(const rdata::msg::LogF64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void rdata::Node::callbackLogI64(const rdata::msg::LogI64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void rdata::Node::callbackLogStr(const rdata::msg::LogStr::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void rdata::Node::callbackLogU64(const rdata::msg::LogU64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}
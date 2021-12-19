#include <RDataNode.hpp>

rdata::Node::Node() : rclcpp_lifecycle::LifecycleNode("rdata")
{
    this->srvCreateLoggerF64 = this->create_service<rdata::srv::CreateLogger>(iface::srv_create_logger_f64,
                                                                              std::bind(&rdata::Node::createLoggerF64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "\033[1;32mCreated service: %s\033[0m", iface::srv_create_logger_f64);

    this->srvRemoveLoggerF64 = this->create_service<rdata::srv::RemoveLogger>(iface::srv_remove_logger_f64,
                                                                              std::bind(&rdata::Node::removeLoggerF64,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "\033[1;32mCreated service: %s\033[0m", iface::srv_remove_logger_f64);
}

rdata::Node::~Node()
{
    RCLCPP_INFO(this->get_logger(), "\033[1;31mDestroying RocketDATA node.\033[0m");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::Node::on_configure(const rclcpp_lifecycle::State &)
{
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initialize and configure our publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Service functions to create a new data logger of a given type
// This logic should be seperated across dedicated services on a per type basis since all requestors
// will know the type of datalogger that they will be requisting at compile time (avoid last minute decision making)
// These functions will append the created subscriber to the approriate vector
// These functions cannot be templated as it would require templated function pointers in the std::bind of the callback function
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

    RCLCPP_INFO(this->get_logger(), "\033[1;32mCreated F64 logger: %s\033[0m", this->loggersF64.back().subPtr->get_topic_name());

    response->completed = true;
}

// Service functions to remove data loggers by topic name
void rdata::Node::removeLoggerF64(const std::shared_ptr<rdata::srv::RemoveLogger::Request> request,
                                  std::shared_ptr<rdata::srv::RemoveLogger::Response> response)
{

    auto loggersTranformed = removeLoggerByTopic(this->loggersF64, request->topic.c_str());
    int loggersRemoved = this->loggersF64.size() - loggersTranformed.size();
    this->loggersF64 = loggersTranformed;

    RCLCPP_INFO(this->get_logger(), "\033[1;31mRemoved %d F64 logger(s) with topic: %s\033[0m", loggersRemoved, request->topic.c_str());

    response->completed = true;
}

// The callbacks needed by the ros2 subscriptions cannot be templated so this must be done through overloading
// The function call 'tryToWriteToInfluxDB()' can be templated however which lets us define uniform error handling logic once for all overloads
void rdata::Node::callbackLogF64(const rdata::msg::LogF64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

// void RocketDataNode::topicCallbackLogFloat64(const rocketdata::msg::LogFloat64::SharedPtr msg)
// {
//     tryToWriteToInfluxDB(msg);
// }

// void RocketDataNode::topicCallbackLogInt64(const rocketdata::msg::LogInt64::SharedPtr msg)
// {
//     tryToWriteToInfluxDB(msg);
// }

// void RocketDataNode::topicCallbackLogString(const rocketdata::msg::LogString::SharedPtr msg)
// {
//     tryToWriteToInfluxDB(msg);
// }

// void RocketDataNode::topicCallbackLogUint64(const rocketdata::msg::LogUint64::SharedPtr msg)
// {
//     tryToWriteToInfluxDB(msg);
// }
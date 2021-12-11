#include <RDataNode.hpp>

rdata::Node::Node() : rclcpp::Node("rdata")
{
    this->srvCreateSubBool = this->create_service<rdata::srv::CreateSub>(RDATA_SRV_CREATE_SUB_BOOL,
                                                                         std::bind(&rdata::Node::createSubBool,
                                                                                   this,
                                                                                   std::placeholders::_1,
                                                                                   std::placeholders::_2));

    this->srvRemoveSubBool = this->create_service<rdata::srv::RemoveSub>(RDATA_SRV_REMOVE_SUB_BOOL,
                                                                         std::bind(&rdata::Node::removeSubBool,
                                                                                   this,
                                                                                   std::placeholders::_1,
                                                                                   std::placeholders::_2));
}

rdata::Node::~Node()
{
    RCLCPP_INFO(this->get_logger(), "\033[1;31mDestroying RocketDATA node.\033[0m");
}

// Service functions to create a new data logger of a given type
// This logic should be seperated across dedicated services on a per type basis since all requestors
// will know the type of datalogger that they will be requisting at compile time (avoid last minute decision making)
// These functions will append the created subscriber to the approriate vector
// These functions cannot be templated as it would require templated function pointers in the std::bind of the callback function
void rdata::Node::createSubBool(const std::shared_ptr<rdata::srv::CreateSub::Request> request,
                                std::shared_ptr<rdata::srv::CreateSub::Response> response)
{
    auto callbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opts = rclcpp::SubscriptionOptions();
    opts.callback_group = callbackGroup;

    auto subPtr = this->create_subscription<rdata::msg::LogBool>(request->topic.c_str(),
                                                                 10,
                                                                 std::bind(&rdata::Node::callbackLogBool, this, std::placeholders::_1),
                                                                 opts);

    rdata::Sub<rdata::msg::LogBool> sub = {
        subPtr,
        callbackGroup,
        opts,
    };

    this->subsBool.push_back(sub);

    response->completed = true;
}

// Service functions to remove data loggers by topic name
void rdata::Node::removeSubBool(const std::shared_ptr<rdata::srv::RemoveSub::Request> request,
                                std::shared_ptr<rdata::srv::RemoveSub::Response> response)
{
    this->subsBool = removeSubByTopic(this->subsBool, request->topic.c_str());

    response->completed = true;
}

// The callbacks needed by the ros2 subscriptions cannot be templated so this must be done through overloading
// The function call 'tryToWriteToInfluxDB()' can be templated however which lets us define uniform error handling logic once for all overloads
void rdata::Node::callbackLogBool(const rdata::msg::LogBool::SharedPtr msg)
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
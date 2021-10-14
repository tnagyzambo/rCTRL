#include <RocketDataNode.hpp>

RocketDataNode::RocketDataNode() : rclcpp::Node("rocketdata")
{
    this->callbackGroupSubcriberLogBool = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->callbackGroupSubcriberLogFloat64 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->callbackGroupSubcriberLogInt64 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->callbackGroupSubcriberLogString = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->callbackGroupSubcriberLogUint64 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->subscriptionLogBoolOpt = rclcpp::SubscriptionOptions();
    this->subscriptionLogFloat64Opt = rclcpp::SubscriptionOptions();
    this->subscriptionLogInt64Opt = rclcpp::SubscriptionOptions();
    this->subscriptionLogStringOpt = rclcpp::SubscriptionOptions();
    this->subscriptionLogUint64Opt = rclcpp::SubscriptionOptions();

    this->subscriptionLogBoolOpt.callback_group = this->callbackGroupSubcriberLogBool;
    this->subscriptionLogFloat64Opt.callback_group = this->callbackGroupSubcriberLogFloat64;
    this->subscriptionLogInt64Opt.callback_group = this->callbackGroupSubcriberLogInt64;
    this->subscriptionLogStringOpt.callback_group = this->callbackGroupSubcriberLogString;
    this->subscriptionLogUint64Opt.callback_group = this->callbackGroupSubcriberLogUint64;

    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGBOOL << "\".\033[0m" << std::endl;
    this->subscriptionLogBool = this->create_subscription<rocketdata::msg::LogBool>(
        ROS_ROCKEDATA_TOPIC_LOGBOOL,
        10,
        std::bind(&RocketDataNode::topicCallbackLogBool, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGFLOAT64 << "\".\033[0m" << std::endl;
    this->subscriptionLogFloat64 = this->create_subscription<rocketdata::msg::LogFloat64>(
        ROS_ROCKEDATA_TOPIC_LOGFLOAT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogFloat64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGINT64 << "\".\033[0m" << std::endl;
    this->subscriptionLogInt64 = this->create_subscription<rocketdata::msg::LogInt64>(
        ROS_ROCKEDATA_TOPIC_LOGINT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogInt64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGSTRING << "\".\033[0m" << std::endl;
    this->subscriptionLogString = this->create_subscription<rocketdata::msg::LogString>(
        ROS_ROCKEDATA_TOPIC_LOGSTRING,
        10,
        std::bind(&RocketDataNode::topicCallbackLogString, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGUINT64 << "\".\033[0m" << std::endl;
    this->subscriptionLogUint64 = this->create_subscription<rocketdata::msg::LogUint64>(
        ROS_ROCKEDATA_TOPIC_LOGUINT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogUint64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    std::cout << std::endl;

    RCLCPP_INFO(this->get_logger(), "\033[1;32mRocketDATA node has successfully constructed.\033[0m");
    RCLCPP_INFO(this->get_logger(), "RocketDATA is writing to InfluxDB bucket: %s", this->influxClient.credentials.bucket.c_str());
}

RocketDataNode::~RocketDataNode()
{
    RCLCPP_INFO(this->get_logger(), "\033[1;31mDestroying RocketDATA node.\033[0m");
}

// The callbacks needed by the ros2 subscriptions cannot be templated so this must be done through overloading
// The function call 'tryToWriteToInfluxDB()' can be templated however which lets us define uniform error handling logic once for all overloads
void RocketDataNode::topicCallbackLogBool(const rocketdata::msg::LogBool::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void RocketDataNode::topicCallbackLogFloat64(const rocketdata::msg::LogFloat64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void RocketDataNode::topicCallbackLogInt64(const rocketdata::msg::LogInt64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void RocketDataNode::topicCallbackLogString(const rocketdata::msg::LogString::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}

void RocketDataNode::topicCallbackLogUint64(const rocketdata::msg::LogUint64::SharedPtr msg)
{
    tryToWriteToInfluxDB(msg);
}
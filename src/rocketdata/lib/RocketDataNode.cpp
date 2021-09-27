#include <RocketDataNode.hpp>

RocketDataNode::RocketDataNode() : rclcpp::Node("rocketdata") {
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

    this->subscriptionLogBool = this->create_subscription<rocketdata::msg::LogBool>(
        ROS_ROCKEDATA_TOPIC_LOGBOOL,
        10,
        std::bind(&RocketDataNode::topicCallbackLogBool, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    this->subscriptionLogFloat64 = this->create_subscription<rocketdata::msg::LogFloat64>(
        ROS_ROCKEDATA_TOPIC_LOGFLOAT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogFloat64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    this->subscriptionLogInt64 = this->create_subscription<rocketdata::msg::LogInt64>(
        ROS_ROCKEDATA_TOPIC_LOGINT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogInt64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    this->subscriptionLogString = this->create_subscription<rocketdata::msg::LogString>(
        ROS_ROCKEDATA_TOPIC_LOGSTRING,
        10,
        std::bind(&RocketDataNode::topicCallbackLogString, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    this->subscriptionLogUint64 = this->create_subscription<rocketdata::msg::LogUint64>(
        ROS_ROCKEDATA_TOPIC_LOGUINT64,
        10,
        std::bind(&RocketDataNode::topicCallbackLogUint64, this, std::placeholders::_1),
        subscriptionLogBoolOpt);

    RCLCPP_INFO(this->get_logger(), "\033[1;35mRocketDATA is subscribing to topic \"%s\".\033[0m", ROS_ROCKEDATA_TOPIC_LOGBOOL);
    std::cout << "\033[1;35mRocketDATA is subscribing to topic \"" << ROS_ROCKEDATA_TOPIC_LOGBOOL << "\".\033[0m" << std::endl;
    std::cout << std::endl;
}

RocketDataNode::~RocketDataNode() {
    
}

void RocketDataNode::topicCallbackLogBool(const rocketdata::msg::LogBool::SharedPtr msg) {
    this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
}

void RocketDataNode::topicCallbackLogFloat64(const rocketdata::msg::LogFloat64::SharedPtr msg) {
    this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
}

void RocketDataNode::topicCallbackLogInt64(const rocketdata::msg::LogInt64::SharedPtr msg) {
    this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
}

void RocketDataNode::topicCallbackLogString(const rocketdata::msg::LogString::SharedPtr msg) {
    this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
}

void RocketDataNode::topicCallbackLogUint64(const rocketdata::msg::LogUint64::SharedPtr msg) {
    this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
}
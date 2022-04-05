#include <rgpio/gpio/virtual.hpp>

rgpio::gpio::Virtual::Virtual(rclcpp::Node *node, std::string name, chip_number chipNumber, line_number lineNumber)
    : node(node), name(name), chipNumber(chipNumber), lineNumber(lineNumber) {
    // The internal vitrual level must be initialize
    this->level = line_level::level(0);

    this->simInTopic = this->createSimInTopic();
    this->simOutTopic = this->createSimOutTopic();

    this->callbackGroupSimInSubscriber = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->subscriptionOptSimInSubscriber = rclcpp::SubscriptionOptions();
    this->subscriptionOptSimInSubscriber.callback_group = this->callbackGroupSimInSubscriber;
    this->simInSubscriber = this->node->create_subscription<rgpio_msgs::msg::SimInput>(
        this->simInTopic,
        10,
        std::bind(&Virtual::topicCallbackSimInSubscriber, this, std::placeholders::_1),
        this->subscriptionOptSimInSubscriber);

    this->simOutPublisher = this->node->create_publisher<rgpio_msgs::msg::SimOutput>(this->simOutTopic, 10);

    RCLCPP_INFO(this->node->get_logger(),
                "Virtual GPIO created with simulation topics '%s' and '%s'",
                this->simInTopic.c_str(),
                this->simOutTopic.c_str());
}

rgpio::gpio::Virtual::~Virtual() {
    RCLCPP_INFO(this->node->get_logger(),
                "Virtual GPIO destroyed with simulation topics '%s' and '%s'",
                this->simInTopic.c_str(),
                this->simOutTopic.c_str());
}

void rgpio::gpio::Virtual::setLineAsInput() {
    RCLCPP_INFO(this->node->get_logger(),
                "Chip '%d' line '%d' has been set as an input (VIRUAL)",
                this->chipNumber.value,
                this->lineNumber.value);
}

void rgpio::gpio::Virtual::setLineAsOutput() {
    RCLCPP_INFO(this->node->get_logger(),
                "Chip '%d' line '%d' has been set as an output (VIRTUAL)",
                this->chipNumber.value,
                this->lineNumber.value);
}

rgpio::gpio::line_level::level rgpio::gpio::Virtual::readLine() { return this->level; }

void rgpio::gpio::Virtual::setLine(line_level::level level) {
    this->level = level;

    auto simOutMsg = rgpio_msgs::msg::SimOutput();
    simOutMsg.level = line_level::toInt(this->level);
    this->simOutPublisher->publish(simOutMsg);
}

std::string rgpio::gpio::Virtual::createSimInTopic() {
    std::string simInTopic = "rgpio_";
    simInTopic.append(this->name);
    simInTopic.append("_sim_input");

    return simInTopic;
}

std::string rgpio::gpio::Virtual::createSimOutTopic() {
    std::string simOutTopic = "rgpio_";
    simOutTopic.append(this->name);
    simOutTopic.append("_sim_output");

    return simOutTopic;
}

void rgpio::gpio::Virtual::topicCallbackSimInSubscriber(const rgpio_msgs::msg::SimInput::SharedPtr msg) {
    this->level = line_level::level(msg->level);
}

#include <rgpio/output.hpp>

rgpio::Output::Output(rclcpp::Node *node, ::toml::node_view<::toml::node> toml, std::string name) {
    this->node = node;
    this->gpio = constructOutput(node, toml, name);
    this->gpio->setLineAsOutput();

    // this->rDataPublisher =
    //     this->node.template create_publisher<rgpio_msgs::msg::LogString>(ROS_ROCKEDATA_TOPIC_LOGSTRING, 10);
}

void rgpio::Output::write(rgpio::gpio::line_level::level level) {
    try {
        this->gpio->setLine(level);
    } catch (rgpio::except::gpio_error &e) {
        RCLCPP_ERROR(this->node->get_logger(), "%s", e.what());

        throw e;
    }

    // // Log to rocketDATA
    // auto rDataMsg = rocketgpio::msg::LogString();
    // rDataMsg.measurment = "GPIO Output";
    // rDataMsg.sensor = this->name;
    // rDataMsg.value = rgpio::gpio::line_level::toStr(level);
    // this->rDataPublisher->publish(rDataMsg);
}

#include <rgpio/input.hpp>

rgpio::Input::Input(rclcpp::Node *node, ::toml::node_view<::toml::node> toml, std::string name) {
    this->node = node;
    this->gpio = constructInput(node, toml, name);
    this->gpio->setLineAsInput();

    // this->rDataPublisher =
    //     this->node.template create_publisher<rgpio_msgs::msg::LogString>(ROS_ROCKEDATA_TOPIC_LOGSTRING, 10);
}

rgpio::gpio::line_level::level rgpio::Input::read() {
    rgpio::gpio::line_level::level level;

    try {
        level = this->gpio->readLine();
    } catch (rgpio::except::gpio_error &e) {
        RCLCPP_ERROR(this->node->get_logger(), "%s", e.what());

        throw e;
    }

    // // Log to rocketDATA
    // auto rDataMsg = rgpio_msgs::msg::LogString();
    // rDataMsg.measurment = "GPIO Input";
    // rDataMsg.sensor = this->name;
    // rDataMsg.value = rgpio::gpio::line_level::toStr(level);
    // this->rDataPublisher->publish(rDataMsg);

    return level;
}

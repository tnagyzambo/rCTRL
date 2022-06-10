#include <rgpio/output.hpp>

rgpio::Output::Output(rclcpp::Node *node, ::toml::node_view<::toml::node> toml, std::string name) {
    this->node = node;
    this->gpio = constructOutput(node, toml, name);
    this->gpio->setLineAsOutput();
}

void rgpio::Output::write(rgpio::gpio::line_level::level level) {
    try {
        this->gpio->setLine(level);
    } catch (rgpio::except::gpio_error &e) {
        RCLCPP_ERROR(this->node->get_logger(), "%s", e.what());

        throw e;
    }
}

// Given the input parameters, construct an appropriate output
// Return pointer to base 'Iface' of the output
std::unique_ptr<rgpio::gpio::Gpio> rgpio::Output::constructOutput(rclcpp::Node *node,
                                                                  ::toml::node_view<::toml::node> toml,
                                                                  std::string name) {
    std::string mode = rutil::toml::getTomlEntryByKey<std::string>(toml, "mode");

    if (mode != "output") {
        std::string error = fmt::format("Config entry does not define an output!\nTOML: {}", toml);

        throw rgpio::except::config_parse_error(error);
    };

    return gpio::constructGpio(node, toml, name);
}

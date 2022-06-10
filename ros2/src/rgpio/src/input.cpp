#include <rgpio/input.hpp>

rgpio::Input::Input(rclcpp::Node *node, ::toml::node_view<::toml::node> toml, std::string name) {
    this->node = node;
    this->gpio = constructInput(node, toml, name);
    this->gpio->setLineAsInput();
}

rgpio::gpio::line_level::level rgpio::Input::read() {
    rgpio::gpio::line_level::level level;

    try {
        level = this->gpio->readLine();
    } catch (rgpio::except::gpio_error &e) {
        RCLCPP_ERROR(this->node->get_logger(), "%s", e.what());

        throw e;
    }

    return level;
}

// Given the input parameters, construct an appropriate input
// Return pointer to base 'Iface' of the input
std::unique_ptr<rgpio::gpio::Gpio> rgpio::Input::constructInput(rclcpp::Node *node,
                                                                ::toml::node_view<::toml::node> toml,
                                                                std::string name) {
    auto gpioView = rutil::toml::viewOfTable(toml, name.c_str());

    std::string mode = rutil::toml::getTomlEntryByKey<std::string>(gpioView, "mode");

    if (mode != "input") {
        std::string error = fmt::format("Config entry does not define an input!\nTOML: {}", gpioView);

        throw rgpio::except::config_parse_error(error);
    };

    return gpio::constructGpio(node, gpioView, name);
}

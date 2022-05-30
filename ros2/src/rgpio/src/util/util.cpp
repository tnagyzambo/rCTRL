#include <rgpio/util/util.hpp>

// Given the input parameters, construct an appropriate input
// Return pointer to base 'Iface' of the input
std::unique_ptr<rgpio::gpio::Gpio> rgpio::constructInput(rclcpp::Node *node,
                                                         ::toml::node_view<::toml::node> toml,
                                                         std::string name) {
    std::string mode = rutil::toml::getTomlEntryByKey<std::string>(toml, "mode");

    if (mode != "input") {
        std::stringstream error;

        error << "Config entry does not define an input\n";
        error << "TOML: " << toml;

        throw rgpio::except::config_parse_error(error.str());
    };

    return constructGpio(node, toml, name);
}

// Given the input parameters, construct an appropriate output
// Return pointer to base 'Iface' of the output
std::unique_ptr<rgpio::gpio::Gpio> rgpio::constructOutput(rclcpp::Node *node,
                                                          ::toml::node_view<::toml::node> toml,
                                                          std::string name) {
    std::string mode = rutil::toml::getTomlEntryByKey<std::string>(toml, "mode");

    if (mode != "output") {
        std::stringstream error;

        error << "Config entry does not define an output\n";
        error << "TOML: " << toml;

        throw rgpio::except::config_parse_error(error.str());
    };

    return constructGpio(node, toml, name);
}

std::unique_ptr<rgpio::gpio::Gpio> rgpio::constructGpio(rclcpp::Node *node,
                                                        ::toml::node_view<::toml::node> toml,
                                                        std::string name) {
    std::unique_ptr<rgpio::gpio::Gpio> gpio;

    rgpio::gpio::chip_number chipNumber =
        rgpio::gpio::chip_number(rutil::toml::getTomlEntryByKey<int>(toml, "chipNumber"));
    rgpio::gpio::line_number lineNumber =
        rgpio::gpio::line_number(rutil::toml::getTomlEntryByKey<int>(toml, "lineNumber"));
    bool sim = rutil::toml::getTomlEntryByKey<bool>(toml, "sim");

    try {
        if (sim == false) {
            gpio = std::make_unique<rgpio::gpio::Real>(node, name, chipNumber, lineNumber);
        } else {
            gpio = std::make_unique<rgpio::gpio::Virtual>(node, name, chipNumber, lineNumber);
        }
    } catch (rgpio::except::gpio_error &e) {
        std::stringstream error;

        error << "Failed to construct gpio!\n";
        error << "Error: " << e.what();

        throw rgpio::except::config_parse_error(error.str());
    }

    return gpio;
}

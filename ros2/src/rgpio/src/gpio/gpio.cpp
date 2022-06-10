#include <rgpio/gpio/gpio.hpp>

std::unique_ptr<rgpio::gpio::Gpio> rgpio::gpio::constructGpio(rclcpp::Node *node,
                                                              ::toml::node_view<::toml::node> toml,
                                                              std::string name) {
    std::unique_ptr<rgpio::gpio::Gpio> gpio;

    rgpio::gpio::chip_number chipNumber =
        rgpio::gpio::chip_number(rutil::toml::getTomlEntryByKey<int>(toml, "chipNumber"));
    rgpio::gpio::line_number lineNumber =
        rgpio::gpio::line_number(rutil::toml::getTomlEntryByKey<int>(toml, "lineNumber"));
    bool sim = rutil::toml::getTomlEntryByKey<bool>(toml, "sim");
    rgpio::gpio::line_level::level defaultLevel =
        (rgpio::gpio::line_level::level)rutil::toml::getTomlEntryByKey<int>(toml, "defaultLevel");

    try {
        if (sim == false) {
            gpio = std::make_unique<rgpio::gpio::Real>(node, name, chipNumber, lineNumber, defaultLevel);
        } else {
            gpio = std::make_unique<rgpio::gpio::Virtual>(node, name, chipNumber, lineNumber, defaultLevel);
        }
    } catch (rgpio::except::gpio_error &e) {
        std::string error = fmt::format("Failed to construct gpio!\nError: ", e.what());

        throw rgpio::except::config_parse_error(error);
    }

    return gpio;
}

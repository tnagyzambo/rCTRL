#include <rgpio/gpio/real.hpp>

rgpio::gpio::Real::Real(
    rclcpp::Node *node, std::string name, chip_number chipNumber, line_number lineNumber, line_level::level defaultLevel)
    : node(node), name(name), chipNumber(chipNumber), lineNumber(lineNumber), defaultLevel(defaultLevel) {
    this->chip = this->getChip(this->chipNumber);
    this->line = this->getLine(this->lineNumber);
}

rgpio::gpio::Real::~Real() { this->releaseLine(); }

struct gpiod_chip *rgpio::gpio::Real::getChip(chip_number chipNumber) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(chipNumber.value);

    if (chip == NULL) {
        std::string error = fmt::format("Chip number '{}' failed to open", chipNumber.value);

        RCLCPP_ERROR(this->node->get_logger(), "%s", error.c_str());

        throw except::gpio_error(error);
    }

    return chip;
}

struct gpiod_line *rgpio::gpio::Real::getLine(line_number lineNumber) {
    struct gpiod_line *line = gpiod_chip_get_line(this->chip, lineNumber.value);

    if (chip == NULL) {
        std::string error = fmt::format("Line number '{}' failed to open", lineNumber.value);

        RCLCPP_ERROR(this->node->get_logger(), "%s", error.c_str());

        throw except::gpio_error(error);
    }

    return line;
}

void rgpio::gpio::Real::setLineAsInput() {
    gpiod_line_request_input(this->line, this->name.c_str());

    RCLCPP_INFO(this->node->get_logger(),
                "Chip '%d' line '%d' has been set as an input",
                this->chipNumber.value,
                this->lineNumber.value);
}

void rgpio::gpio::Real::setLineAsOutput() {
    gpiod_line_request_output(this->line, this->name.c_str(), (int)this->defaultLevel);

    RCLCPP_INFO(this->node->get_logger(),
                "Chip '%d' line '%d' has been set as an output",
                this->chipNumber.value,
                this->lineNumber.value);
}

rgpio::gpio::line_level::level rgpio::gpio::Real::readLine() {
    int level = gpiod_line_get_value(this->line);

    if (level == -1) {
        std::string error = fmt::format("Error: rgpio readLine() failed!\nChip number: {}\nLine number: {}",
                                        this->chipNumber.value,
                                        this->lineNumber.value);

        RCLCPP_ERROR(this->node->get_logger(), "%s", error.c_str());

        throw except::gpio_error(error);
    }

    return line_level::level(level);
}

void rgpio::gpio::Real::setLine(line_level::level level) {
    int success = gpiod_line_set_value(this->line, line_level::toInt(level));

    if (success == -1) {
        std::string error = fmt::format("Error: rgpio setLine() failed!\nChip number: {}\nLine number: {}",
                                        this->chipNumber.value,
                                        this->lineNumber.value);

        RCLCPP_ERROR(this->node->get_logger(), "%s", error.c_str());

        throw except::gpio_error(error);
    }
}

void rgpio::gpio::Real::releaseLine() {
    gpiod_line_release(this->line);

    RCLCPP_INFO(
        this->node->get_logger(), "Chip '%d' line '%d' has been released", this->chipNumber.value, this->lineNumber.value);
}

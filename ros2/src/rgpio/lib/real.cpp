#include <real.hpp>

rgpio::gpio::Real::Real(rclcpp::Node &node, std::string name, chip_number chipNumber, line_number lineNumber)
    : node(node), name(name), chipNumber(chipNumber), lineNumber(lineNumber) {
    this->chip = this->getChip(this->chipNumber);
    this->line = this->getLine(this->lineNumber);
}

rgpio::gpio::Real::~Real() { this->releaseLine(); }

struct gpiod_chip *rgpio::gpio::Real::getChip(chip_number chipNumber) {
    struct gpiod_chip *chip = gpiod_chip_open_by_number(chipNumber.value);

    // Unrecoverable error
    if (chip == NULL) {
        std::stringstream error;

        error << "\033[1;31mGPIO ERROR!\033[0m\n";
        error << "\033[1mError: chip number '" << chipNumber.value << "' failed to open\n";

        RCLCPP_FATAL(this->node.get_logger(), "%s", error.str().c_str());

        throw std::runtime_error(error.str());
    }

    return chip;
}

struct gpiod_line *rgpio::gpio::Real::getLine(line_number lineNumber) {
    struct gpiod_line *line = gpiod_chip_get_line(this->chip, lineNumber.value);

    // Unrecoverable error
    if (chip == NULL) {
        std::stringstream error;

        error << "\033[1;31mGPIO ERROR!\033[0m\n";
        error << "\033[1mError: line number '" << lineNumber.value << "' failed to open\n";

        RCLCPP_FATAL(this->node.get_logger(), "%s", error.str().c_str());

        throw std::runtime_error(error.str());
    }

    return line;
}

void rgpio::gpio::Real::setLineAsInput() {
    RCLCPP_INFO(this->node.get_logger(),
                "Chip '%d' line '%d' has been set as an input",
                this->chipNumber.value,
                this->lineNumber.value);
}

void rgpio::gpio::Real::setLineAsOutput() {
    RCLCPP_INFO(this->node.get_logger(),
                "Chip '%d' line '%d' has been set as an output",
                this->chipNumber.value,
                this->lineNumber.value);
}

rgpio::gpio::line_level::level rgpio::gpio::Real::readLine() {
    int level = gpiod_line_get_value(this->line);

    if (level == -1) {
        throw IOException();
    }

    return line_level::level(level);
}

void rgpio::gpio::Real::setLine(line_level::level level) {
    int success = gpiod_line_set_value(this->line, line_level::toInt(level));

    if (success == -1) {
        throw IOException();
    }
}

void rgpio::gpio::Real::releaseLine() {
    gpiod_line_release(this->line);

    RCLCPP_INFO(
        this->node.get_logger(), "Chip '%d' line '%d' has been released", this->chipNumber.value, this->lineNumber.value);
}

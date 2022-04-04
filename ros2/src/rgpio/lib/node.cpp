#include <node.hpp>

rgpio::Node::Node() : rclcpp::Node("rocketgpio") {
    toml::table tbl = getToml(GPIO_CONFIG_FILE);
    this->gpio = getGpio(tbl);

    RCLCPP_INFO(this->get_logger(), "\033[1;32mrocketGPIO has successfully constructed.\033[0m");
}

rgpio::Node::~Node() {}

// Read GPIO.toml config file and return toml data object
toml::table rgpio::Node::getToml(std::string path) {
    toml::table tbl;

    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error &e) {
        // Unrecoverable error on failure to parse
        std::stringstream error;

        error << "\0337\033[1;31mCANNOT PARSE CONFIG FILE!\033[0m";
        error << "\0338\f\0337\033[1;31mFile Path: \033[0m\033[31m" << GPIO_CONFIG_FILE << "\033[0m";
        error << "\0338\f\033[1;31mError: \033[0m\033[31m" << e << "\033[0m";

        RCLCPP_FATAL(this->get_logger(), "%s", error.str().c_str());

        throw std::runtime_error(error.str());
    }

    return tbl;
}

// This function takes an input table that contains an array of tables with the key 'gpio'
// It traverses that array of tables by casting to the base toml++ type of 'node'
// and then downcasts to a table using 'as_table()' when accessing a particular entry
// This function constructs a vector of pointers to the base GPIO interface class 'Iface' to allow
// for collecting multiple different GPIO types in one vector
std::vector<std::unique_ptr<rgpio::gpio::Iface>> rgpio::Node::getGpio(toml::table tbl) {
    std::vector<std::unique_ptr<rgpio::gpio::Iface>> gpio;
    toml::array *gpioToml = tbl["gpio"].as_array();

    for (toml::node &gpioDefinition : *gpioToml) {
        gpio.push_back(this->constructGpio(*gpioDefinition.as_table()));
    }

    return gpio;
}

// This function reads all the relevant entries in one gpio definition and then constaructs the appropriate GPIO type
// It only return a pointer to the base 'Iface' class of the GPIO however
std::unique_ptr<rgpio::gpio::Iface> rgpio::Node::constructGpio(toml::table tbl) {
    std::unique_ptr<rgpio::gpio::Iface> gpio;

    std::string name = getTomlEntryByKey<std::string>(tbl, "name");
    rgpio::gpio::chip_number chipNumber = rgpio::gpio::chip_number(getTomlEntryByKey<int>(tbl, "chipNumber"));
    rgpio::gpio::line_number lineNumber = rgpio::gpio::line_number(getTomlEntryByKey<int>(tbl, "lineNumber"));
    std::string mode = getTomlEntryByKey<std::string>(tbl, "mode");
    bool sim = getTomlEntryByKey<bool>(tbl, "sim");

    if (mode == "input") {
        gpio = constructInput(name, chipNumber, lineNumber, sim);
    } else if (mode == "output") {
        gpio = constructOutput(name, chipNumber, lineNumber, sim);
    } else {
        // Unrecoverable error
        std::stringstream error;

        error << "\0337\033[1;31mCANNOT PARSE CONFIG FILE!\033[0m";
        error << "\0338\f\0337\033[1;31mFile Path: \033[0m\033[31m" << GPIO_CONFIG_FILE << "\033[0m";
        error << "\0338\f\033[1;31mError:\033[0m\033[31m 'mode' key malformed for one or more GPIO definitions.\033[0m";

        RCLCPP_FATAL(this->get_logger(), "%s", error.str().c_str());

        throw std::runtime_error(error.str());
    }

    return gpio;
}

// Given the input parameters, construct an appropriate input
// Return pointer to base 'Iface' of the input
// NOTE: the parent ROS2 node of 'RocketGpioNode' in passed to the consructor of the GPIO via upcasting
std::unique_ptr<rgpio::gpio::Iface> rgpio::Node::constructInput(std::string name,
                                                                rgpio::gpio::chip_number chipNumber,
                                                                rgpio::gpio::line_number lineNumber,
                                                                bool sim) {
    std::unique_ptr<rgpio::gpio::Iface> gpio;
    rclcpp::Node &node = *this;

    if (sim == false) {
        gpio = std::make_unique<rgpio::gpio::Input<rgpio::gpio::Real>>(node, name, chipNumber, lineNumber);
    } else {
        gpio = std::make_unique<rgpio::gpio::Input<rgpio::gpio::Virtual>>(node, name, chipNumber, lineNumber);
    }

    return gpio;
}

// Given the input parameters, construct an appropriate output
// Return pointer to base 'Iface' of the output
std::unique_ptr<rgpio::gpio::Iface> rgpio::Node::constructOutput(std::string name,
                                                                 rgpio::gpio::chip_number chipNumber,
                                                                 rgpio::gpio::line_number lineNumber,
                                                                 bool sim) {
    std::unique_ptr<rgpio::gpio::Iface> gpio;
    rclcpp::Node &node = *this;

    if (sim == false) {
        gpio = std::make_unique<rgpio::gpio::Output<rgpio::gpio::Real>>(node, name, chipNumber, lineNumber);
    } else {
        gpio = std::make_unique<rgpio::gpio::Output<rgpio::gpio::Virtual>>(node, name, chipNumber, lineNumber);
    }

    return gpio;
}

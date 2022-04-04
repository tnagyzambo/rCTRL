template <class T>
rgpio::gpio::Output<T>::Output(rclcpp::Node &node, std::string name, chip_number chipNumber, line_number lineNumber)
    : T(node, name, chipNumber, lineNumber), name(name), node(node) {
    this->setLineAsOutput();

    // this->rDataPublisher =
    //     this->node.template create_publisher<rgpio_msgs::msg::LogString>(ROS_ROCKEDATA_TOPIC_LOGSTRING, 10);

    this->serviceNameWrite = this->createServiceNameWrite();
    this->service = this->node.template create_service<rgpio_msgs::srv::OutputWrite>(
        serviceNameWrite.c_str(),
        std::bind(&rgpio::gpio::Output<T>::write, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->node.get_logger(),
                "\033[1;32mService '%s' has successfully constructed\033[0m",
                this->serviceNameWrite.c_str());
}

template <class T>
void rgpio::gpio::Output<T>::write(const std::shared_ptr<rgpio_msgs::srv::OutputWrite::Request> request,
                                   std::shared_ptr<rgpio_msgs::srv::OutputWrite::Response> response) {
    // Set the line
    rgpio::gpio::line_level::level level = (rgpio::gpio::line_level::level)request->level;

    try {
        this->setLine(level);
    } catch (rgpio::gpio::IOException &e) {
        // Log to ROS2
        RCLCPP_ERROR(this->node.get_logger(),
                     "\033[1;'%s'\033[0m\033[31m has failed to set '%s'\033[0m \n'%s'",
                     this->name.c_str(),
                     rgpio::gpio::line_level::toStr(level),
                     e.what());

        // Complete request
        response->completed = false;

        return;
    }

    // // Log to rocketDATA
    // auto rDataMsg = rocketgpio::msg::LogString();
    // rDataMsg.measurment = "GPIO Output";
    // rDataMsg.sensor = this->name;
    // rDataMsg.value = rgpio::gpio::line_level::toStr(level);
    // this->rDataPublisher->publish(rDataMsg);

    // Log to ROS2
    RCLCPP_INFO(this->node.get_logger(), "'%s' set '%s'.", this->name.c_str(), rgpio::gpio::line_level::toStr(level));

    // Complete request
    response->completed = true;
}

template <class T>
std::string rgpio::gpio::Output<T>::createServiceNameWrite() {
    std::string serviceNameWrite = "rGPIO_";
    serviceNameWrite.append(this->name);
    serviceNameWrite.append("_write");

    return serviceNameWrite;
}

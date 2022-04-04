template <class T>
rgpio::gpio::Input<T>::Input(rclcpp::Node &node, std::string name, chip_number chipNumber, line_number lineNumber)
    : T(node, name, chipNumber, lineNumber), name(name), node(node) {
    // Call inherited method to set line as input
    // I think the compiler checks all possible instances of the templated base class to make sure
    // that it implements the method
    this->setLineAsInput();

    // this->rDataPublisher =
    //     this->node.template create_publisher<rgpio_msgs::msg::LogString>(ROS_ROCKEDATA_TOPIC_LOGSTRING, 10);

    this->serviceNameRead = this->createServiceNameRead();

    // This is a bit of a complex line
    // The input class creates and takes ownership of a ROS2 service
    // This service is constructed by taking a pointer to the parent ROS2 node and calling its member function
    // `create_service<>()` create_service<>() must take a call back function that it will call when a request for
    // service is recieved by ROS2 In this case we a using a member function of a particular input class in order to
    // properly access the defined gpiod properties of the input Since we are trying to use a member function as a
    // callback we cannot simply pass a pointer to the function as `&this->doInputThing` This is because a pointer to a
    // member function actually consists of two pointers, one that points to the function and one that points to the
    // particular instance of the class that it is a member of To do this then std::bind() must be used as intermediate
    // step to generate a pointer that can be used as a callback for the service Note that `std::bind` takes placeholder
    // arguments that corrsepond to the number of arguments that the member function takes Now for the final bit of
    // confusion Normally this entire mess would just look like `this->node.create_service<>()` However since the input
    // class whos member function we are using as callback is a templated class (to account for both real and virtual
    // inputs) The compiler requires an explicit flag to be notified of this This is where the `template` in
    // `this->node.template create_service<>()` comes from, it is not atually member of `this->node`, just a compiler
    // flag
    this->service = this->node.template create_service<rgpio_msgs::srv::InputRead>(
        serviceNameRead.c_str(),
        std::bind(&rgpio::gpio::Input<T>::read, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->node.get_logger(),
                "\033[1;32mService '%s' has successfully constructed\033[0m",
                this->serviceNameRead.c_str());
}

template <class T>
void rgpio::gpio::Input<T>::read(const std::shared_ptr<rgpio_msgs::srv::InputRead::Request> request,
                                 std::shared_ptr<rgpio_msgs::srv::InputRead::Response> response) {
    // Discard the empty request
    (void)request;
    rgpio::gpio::line_level::level level;

    try {
        level = this->readLine();
    } catch (rgpio::gpio::IOException &e) {
        // Log to ROS2
        RCLCPP_ERROR(this->node.get_logger(),
                     "\033[1;31m'%s'\033[0m\033[31m has failed to read\033[0m \n'%s'",
                     this->name.c_str(),
                     e.what());

        // Set level anyway to avoid NULL data
        // Complete request
        response->level = line_level::level(0);
        response->completed = false;

        return;
    }

    // // Log to rocketDATA
    // auto rDataMsg = rgpio_msgs::msg::LogString();
    // rDataMsg.measurment = "GPIO Input";
    // rDataMsg.sensor = this->name;
    // rDataMsg.value = rgpio::gpio::line_level::toStr(level);
    // this->rDataPublisher->publish(rDataMsg);

    // Log to ROS2
    RCLCPP_INFO(this->node.get_logger(), "'%s' read '%s'.", this->name.c_str(), rgpio::gpio::line_level::toStr(level));

    // Complete request
    response->level = line_level::toInt(level);
    response->completed = true;
}

template <class T>
std::string rgpio::gpio::Input<T>::createServiceNameRead() {
    std::string serviceNameWrite = "rGPIO_";
    serviceNameRead.append(this->name);
    serviceNameRead.append("_read");

    return serviceNameRead;
}

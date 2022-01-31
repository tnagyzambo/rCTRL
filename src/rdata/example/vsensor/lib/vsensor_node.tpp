template <typename T>
rdata::vsensor::Node<T>::Node(const char *nodeName, std::chrono::milliseconds period) : rclcpp_lifecycle::LifecycleNode(nodeName)
{
    this->nodeName = nodeName;
    this->loggerTopicName = createLoggerTopicName();
    this->period = period;

    // Construct timer and immediately stop it
    this->timer = this->create_wall_timer(period, std::bind(&rdata::vsensor::Node<T>::timerCallback, this));
    this->timer->cancel();
}

template <typename T>
rdata::vsensor::Node<T>::~Node<T>()
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::destructing);
}

// Callback to execute upon receiving configure command
// If this returns SUCCESS the node will enter the inactive state
// If this returns FAILURE the node will return to the unconfigured state
// If this returns ERROR the node run the errorProcessing callback
template <typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::vsensor::Node<T>::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::configuring);

    try
    {
        rdata::iface::createLogger(this->clCreateLogger->get_service_name(), this->get_node_base_interface(), this->clCreateLogger, this->loggerTopicName.c_str());
    }
    catch (const rutil::except::service_error &e)
    {
        // Unrecoverable error on failure to parse
        std::string error = "\033[1;31mFAILED TO CONSTRUCT NODE '";
        error.append(nodeName);
        error.append("'!\033[0m");

        RCLCPP_FATAL(this->get_logger(), "%s", error.c_str());

        throw std::runtime_error(error);
    }

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::inactive);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving activate command
// If this returns SUCCESS the node will enter the active state
// If this returns FAILURE the node will return to the inactive state
// If this returns ERROR the node run the errorProcessing callback
template <typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::vsensor::Node<T>::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::activating);
    // Cannot activate subscribers
    // REFERENCE: https://github.com/ros2/demos/issues/488

    // Start timer
    this->timer->reset();

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::active);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving deactivate command
// If this returns SUCCESS the node will enter the inactive state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
template <typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::vsensor::Node<T>::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::deactivating);
    // Cannot deactivate subscribers
    // REFERENCE: https://github.com/ros2/demos/issues/488

    // Stop timer
    this->timer->cancel();

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::inactive);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving cleanup command
// If this returns SUCCESS the node will enter the unconfigured state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
// Drop all smart pointers and reset influx client
template <typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::vsensor::Node<T>::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::cleaningUp);

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::unconfigured);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Callback to execute upon receiving cleanup command
// If this returns SUCCESS the node will enter the finalized state
// FAILURE is not a valid return during this transition
// If this returns ERROR the node run the errorProcessing callback
// Drop all smart pointers
template <typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rdata::vsensor::Node<T>::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::transition::shuttingDown);

    // Stop timer
    this->timer->cancel();

    RCLCPP_INFO(this->get_logger(), "%s", rctrl::util::fmt::state::finalized);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template <typename T>
uint rdata::vsensor::Node<T>::calcElapsedTime()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->startTime).count();

    return ms;
}

template <typename T>
void rdata::vsensor::Node<T>::timerCallback()
{
}

template <typename T>
std::string rdata::vsensor::Node<T>::createLoggerTopicName()
{
    std::string loggerTopicName = "log_virtual_sensor_";
    loggerTopicName.append(this->nodeName);

    return loggerTopicName;
}

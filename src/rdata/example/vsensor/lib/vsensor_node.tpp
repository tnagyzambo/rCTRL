template <typename T>
rdata::vsensor::Node<T>::Node(const char *nodeName, std::chrono::milliseconds period) : rclcpp::Node(nodeName)
{
    this->nodeName = nodeName;
    this->loggerTopicName = createLoggerTopicName();
    this->period = period;

    this->timer = this->create_wall_timer(period, std::bind(&rdata::vsensor::Node<T>::timerCallback, this));
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
template <typename T>
void rdata::Node::tryToWriteToInfluxDB(T msg)
{
    try
    {
        this->influxClient->writeToInflux(msg->measurment, msg->sensor, msg->value);
    }
    catch (influx::except::PostReq &e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    catch (influx::except::Curl &e)
    {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        exit(1);
    }
}

// Removes loggers from the input vector of loggers based on topic name
// Note that the input is passed in by value so if the function is called as
// 'this->loggers = removeLoggerByTopic(this->loggers, topicName)'
// the SharedPtrs that have been excluded from the return output should naturally go out of scope
// and be dropped provided that the subscribers were only owned by 'this'
// There is no explicit way to delete ROS2 subscribers, but this method seems to be enough
template <typename T>
std::vector<typename rdata::Logger<T>> rdata::Node::removeLoggerByTopic(std::vector<typename rdata::Logger<T>> loggers, const char *topicName)
{
    std::vector<typename rdata::Logger<T>> loggersTransformed;

    for (typename rdata::Logger<T> logger : loggers)
    {
        if ((*logger.subPtr).get_topic_name() != topicName)
        {
            loggersTransformed.push_back(logger);
        }
    }

    return loggersTransformed;
}
// template <typename T>
// void RocketDataNode::tryToWriteToInfluxDB(T msg)
// {
//     try
//     {
//         this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
//     }
//     catch (influxclient::PostRequestException &e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "%s", e.what());
//     }
//     catch (influxclient::CurlException &e)
//     {
//         RCLCPP_FATAL(this->get_logger(), "%s", e.what());
//         exit(1);
//     }
// }

// Removes subscribers from the input vector of subscribers based on topic name
// Note that the input is passed in by value so if the function is called as
// 'this->subs = removeSubByTopic(this->subs, topicName)'
// the SharedPtrs that have been excluded from the return output should naturally go out of scope
// and be dropped provided that the subscribers were only owned by 'this'
// There is no explicit way to delete ROS2 subscribers, but this method seems to be enough
template <typename T>
std::vector<typename rdata::Sub<T>> rdata::Node::removeSubByTopic(std::vector<typename rdata::Sub<T>> subs, const char *topicName)
{
    std::vector<typename rdata::Sub<T>> subsTransformed;

    for (typename rdata::Sub<T> sub : subs)
    {
        if ((*sub.subPtr).get_topic_name() != topicName)
        {
            subsTransformed.push_back(sub);
        }
    }

    return subsTransformed;
}
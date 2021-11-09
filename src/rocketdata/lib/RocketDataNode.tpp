template <typename T>
void RocketDataNode::tryToWriteToInfluxDB(T msg)
{
    try
    {
        this->influxClient.writeToInflux(msg->measurment, msg->sensor, msg->value);
    }
    catch (influxclient::PostRequestException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    catch (influxclient::CurlException &e)
    {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        exit(1);
    }
}
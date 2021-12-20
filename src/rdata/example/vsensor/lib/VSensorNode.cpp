#include <VSensorNode.hpp>

// Base class
// VSensorNode::VSensorNode(std::string nodeName, std::chrono::milliseconds period) : Node(nodeName)
// {
//     this->clCreatelogger = this->create_client<rdata::srv::CreateLogger>(RDATA_SRV_CREATE_LOGGER_F64);

//     auto request = std::make_shared<rdata::srv::CreateLogger::Request>();
//     request->topic = nodeName.c_str();

//     while (!this->clCreatelogger->wait_for_service(1s))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//         }
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//     }

//     auto result = this->clCreatelogger->async_send_request(request);

//     // Wait for the result.
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DONE");
//     }
//     else
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//     }

//     this->nodeName = nodeName;
//     this->period = period;

//     this->timer = this->create_wall_timer(period, std::bind(&VSensorNode::timerCallback, this));
// }

// uint VSensorNode::calcElapsedTime()
// {
//     auto now = std::chrono::high_resolution_clock::now();
//     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->startTime).count();

//     return ms;
// }

// void VSensorNode::timerCallback()
// {
// }

// // Virtual boolean sensor
// VirtualSensorNodeBool::VirtualSensorNodeBool(std::string nodeName, std::chrono::milliseconds period) : VirtualSensorNode(nodeName, period)
// {
//     this->publisher = this->create_publisher<rocketdata::msg::LogBool>(ROS_ROCKEDATA_TOPIC_LOGBOOL, 10);

//     RCLCPP_INFO(this->get_logger(), "Creating virtual boolean sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// VirtualSensorNodeBool::~VirtualSensorNodeBool()
// {
//     RCLCPP_INFO(this->get_logger(), "Destroying virtual boolean sensor \"'%s'\" with period \"'%sms'\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// void VirtualSensorNodeBool::timerCallback()
// {
//     auto message = rocketdata::msg::LogBool();
//     message.measurment = "virtual_bool";
//     message.sensor = this->nodeName;

//     if (this->prevOutput == true)
//     {
//         message.value = false;
//     }
//     else
//     {
//         message.value = true;
//     }

//     this->publisher->publish(message);
//     this->prevOutput = message.value;
// }

// Virtual float sensor
rdata::vsensor::F64::F64(const char *nodeName, std::chrono::milliseconds period) : rdata::vsensor::Node<rdata::msg::LogF64>(nodeName, period)
{
    this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_f64);

    try
    {
        rdata::iface::createLogger(rdata::iface::srv_create_logger_f64, this->get_node_base_interface(), this->clCreateLogger, this->loggerTopicName.c_str());
    }
    catch (const rdata::iface::service_error &e)
    {
        // Unrecoverable error on failure to parse
        std::string error = "\033[1;31mFAILED TO CONSTRUCT NODE '";
        error.append(nodeName);
        error.append("'!\033[0m");

        RCLCPP_FATAL(this->get_logger(), "%s", error.c_str());

        throw std::runtime_error(error);
    }

    this->logger = this->create_publisher<rdata::msg::LogF64>(this->loggerTopicName, 10);
    this->sinPeriod = 2 * 3.14 / (100 * period.count());

    RCLCPP_INFO(this->get_logger(), "\033[1;32mCreated virtual f64 sensor with period \"%sms\"\033[0m", std::to_string(period.count()).c_str());
}

rdata::vsensor::F64::~F64()
{
    this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_f64);

    try
    {
        rdata::iface::removeLogger(rdata::iface::srv_remove_logger_f64, this->get_node_base_interface(), this->clRemoveLogger, this->loggerTopicName.c_str());
    }
    catch (const rdata::iface::service_error &e)
    {
        // Unrecoverable error on failure to parse
        std::string error = "\033[1;31mFAILED TO DESTRUCT NODE '";
        error.append(nodeName);
        error.append("'!\033[0m");

        RCLCPP_FATAL(this->get_logger(), "%s", error.c_str());

        throw std::runtime_error(error);
    }

    RCLCPP_INFO(this->get_logger(), "Destroyed virtual f64 sensor with period \"%sms\".", std::to_string(period.count()).c_str());
}

void rdata::vsensor::F64::timerCallback()
{
    auto message = rdata::msg::LogF64();
    uint ms = this->calcElapsedTime();

    message.measurment = "virtual_float";
    message.sensor = this->nodeName;
    message.value = sin(this->sinPeriod * ms);

    this->logger->publish(message);
}

// // Virtual int sensor
// VirtualSensorNodeInt64::VirtualSensorNodeInt64(std::string nodeName, std::chrono::milliseconds period) : VirtualSensorNode(nodeName, period)
// {
//     this->publisher = this->create_publisher<rocketdata::msg::LogInt64>(ROS_ROCKEDATA_TOPIC_LOGINT64, 10);

//     RCLCPP_INFO(this->get_logger(), "Creating virtual int64 sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// VirtualSensorNodeInt64::~VirtualSensorNodeInt64()
// {
//     RCLCPP_INFO(this->get_logger(), "Destroying virtual int64 sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// void VirtualSensorNodeInt64::timerCallback()
// {
//     auto message = rocketdata::msg::LogInt64();

//     message.measurment = "virtual_int";
//     message.sensor = this->nodeName;
//     message.value = rand() % 10;

//     this->publisher->publish(message);
// }

// // Virtual string sensor
// VirtualSensorNodeString::VirtualSensorNodeString(std::string nodeName, std::chrono::milliseconds period) : VirtualSensorNode(nodeName, period)
// {
//     this->publisher = this->create_publisher<rocketdata::msg::LogString>(ROS_ROCKEDATA_TOPIC_LOGSTRING, 10);

//     RCLCPP_INFO(this->get_logger(), "Creating virtual string sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// VirtualSensorNodeString::~VirtualSensorNodeString()
// {
//     RCLCPP_INFO(this->get_logger(), "Destroying virtual string sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// void VirtualSensorNodeString::timerCallback()
// {
//     auto message = rocketdata::msg::LogString();
//     uint ms = VirtualSensorNode::calcElapsedTime();

//     message.measurment = "virtual_string";
//     message.sensor = this->nodeName;
//     message.value = "Event at " + std::to_string(ms) + "ms";

//     this->publisher->publish(message);
// }

// // Virtual uint sensor
// VirtualSensorNodeUInt64::VirtualSensorNodeUInt64(std::string nodeName, std::chrono::milliseconds period) : VirtualSensorNode(nodeName, period)
// {
//     this->publisher = this->create_publisher<rocketdata::msg::LogUint64>(ROS_ROCKEDATA_TOPIC_LOGUINT64, 10);

//     RCLCPP_INFO(this->get_logger(), "Creating virtual uint64 sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// VirtualSensorNodeUInt64::~VirtualSensorNodeUInt64()
// {
//     RCLCPP_INFO(this->get_logger(), "Destroying virtual uint64 sensor \"%s\" with period \"%sms\".", this->nodeName.c_str(), std::to_string(period.count()).c_str());
// }

// void VirtualSensorNodeUInt64::timerCallback()
// {
//     auto message = rocketdata::msg::LogUint64();

//     message.measurment = "virtual_uint";
//     message.sensor = this->nodeName;
//     message.value = rand() % 10 + 4000000000;

//     this->publisher->publish(message);
// }

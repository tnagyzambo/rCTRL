#include <vsensor_node.hpp>

namespace rdata::vsensor {
    // Virtual boolean sensor
    Bool::Bool(const char *nodeName, std::chrono::milliseconds period) : Node<rdata::msg::LogBool>(nodeName, period) {
        this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_bool);
        this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_bool);
        this->logger = this->create_publisher<rdata::msg::LogBool>(this->loggerTopicName, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual bool sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    Bool::~Bool() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual bool sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    void Bool::timerCallback() {
        auto message = rdata::msg::LogBool();
        message.measurment = "virtual_bool";
        message.sensor = this->nodeName;
        message.value = !this->prevOutput;

        this->logger->publish(message);
        this->prevOutput = message.value;
    }

    // Virtual float sensor
    F64::F64(const char *nodeName, std::chrono::milliseconds period) : Node<rdata::msg::LogF64>(nodeName, period) {
        this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_f64);
        this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_f64);
        this->logger = this->create_publisher<rdata::msg::LogF64>(this->loggerTopicName, 10);
        this->sinPeriod = 2 * 3.14 / (100 * period.count());

        RCLCPP_INFO(
            this->get_logger(), "Created virtual f64 sensor with period '%sms'", std::to_string(period.count()).c_str());
    }

    F64::~F64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual f64 sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    void F64::timerCallback() {
        auto message = rdata::msg::LogF64();
        uint ms = this->calcElapsedTime();

        message.measurment = "virtual_float";
        message.sensor = this->nodeName;
        message.value = sin(this->sinPeriod * ms);

        this->logger->publish(message);
    }

    // Virtual int sensor
    I64::I64(const char *nodeName, std::chrono::milliseconds period) : Node<rdata::msg::LogI64>(nodeName, period) {
        this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_i64);
        this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_i64);
        this->logger = this->create_publisher<rdata::msg::LogI64>(this->loggerTopicName, 10);

        RCLCPP_INFO(
            this->get_logger(), "Created virtual i64 sensor with period '%sms'", std::to_string(period.count()).c_str());
    }

    I64::~I64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual i64 sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    void I64::timerCallback() {
        auto message = rdata::msg::LogI64();

        message.measurment = "virtual_int";
        message.sensor = this->nodeName;
        message.value = rand() % 10;

        this->logger->publish(message);
    }

    // Virtual string sensor
    Str::Str(const char *nodeName, std::chrono::milliseconds period) : Node<rdata::msg::LogStr>(nodeName, period) {
        this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_str);
        this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_str);
        this->logger = this->create_publisher<rdata::msg::LogStr>(this->loggerTopicName, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Created virtual string sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    Str::~Str() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual string sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    void Str::timerCallback() {
        auto message = rdata::msg::LogStr();
        uint ms = this->calcElapsedTime();

        message.measurment = "virtual_string";
        message.sensor = this->nodeName;
        message.value = "Event at " + std::to_string(ms) + "ms";

        this->logger->publish(message);
    }

    // Virtual uint sensor
    U64::U64(const char *nodeName, std::chrono::milliseconds period) : Node<rdata::msg::LogU64>(nodeName, period) {
        this->clCreateLogger = this->create_client<rdata::srv::CreateLogger>(rdata::iface::srv_create_logger_u64);
        this->clRemoveLogger = this->create_client<rdata::srv::RemoveLogger>(rdata::iface::srv_remove_logger_u64);
        this->logger = this->create_publisher<rdata::msg::LogU64>(this->loggerTopicName, 10);

        RCLCPP_INFO(
            this->get_logger(), "Created virtual u64 sensor with period '%sms'", std::to_string(period.count()).c_str());
    }

    U64::~U64() {
        RCLCPP_INFO(this->get_logger(),
                    "Destroyed virtual u64 sensor with period '%sms'",
                    std::to_string(period.count()).c_str());
    }

    void U64::timerCallback() {
        auto message = rdata::msg::LogU64();

        message.measurment = "virtual_uint";
        message.sensor = this->nodeName;
        message.value = rand() % 10 + 4000000000;

        this->logger->publish(message);
    }
} // namespace rdata::vsensor

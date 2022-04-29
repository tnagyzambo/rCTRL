#include <node.hpp>

namespace recu {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("recu") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->read_timer = this->create_wall_timer(1ms, std::bind(&Node::serialRead, this));
        this->read_timer->cancel();

        this->clCreateLogger = this->create_client<rdata_msgs::srv::CreateLogger>(rdata::iface::srv_create_logger_f64);
        this->clRemoveLogger = this->create_client<rdata_msgs::srv::RemoveLogger>(rdata::iface::srv_remove_logger_f64);
        this->logger = this->create_publisher<rdata_msgs::msg::LogF64>("recu/logger", 10);

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());
    }

    Node::~Node() {}

    LifecycleCallbackReturn Node::on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        this->serial_port = open("/dev/ttyACM0", O_RDWR);

        // Check for errors
        if (this->serial_port < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s\n", errno, strerror(errno));
            return LifecycleCallbackReturn::FAILURE;
        }

        // Serial config struct
        struct termios tty;

        // Read in existing settings
        if (tcgetattr(this->serial_port, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return LifecycleCallbackReturn::FAILURE;
        }

        // PARENB (Parity)
        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        // CSTOPB (Num. Stop Bits)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        // Number Of Bits Per Byte
        tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
        tty.c_cflag |= CS8;    // 8 bits per byte (most common)
        // Flow Control(CRTSCTS)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        // CREAD and CLOCAL
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        // Local Modes (c_lflag)
        tty.c_lflag &= ~ICANON; // Disable in canonical mode, input is processed when a new line character is received.
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
        tty.c_lflag &= ~IEXTEN;
        // Input Modes(c_iflag)
        tty.c_iflag &= ~(IXON); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                         ICRNL); // Disable any special handling of received bytes
        // Output Modes (c_oflag)
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // VMIN and VTIME (c_cc)
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
        // Baud Rate
        cfsetspeed(&tty, B9600);

        // Save tty settings
        if (tcsetattr(this->serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return LifecycleCallbackReturn::FAILURE;
        }

        this->srvMV1_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv1/open", std::bind(&Node::MV1_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/open").c_str());

        this->srvMV1_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv1/close", std::bind(&Node::MV1_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/close").c_str());

        this->srvMV1_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/mv1/get_state", std::bind(&Node::MV1_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1/get_state").c_str());

        this->srvMV2_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv2/open", std::bind(&Node::MV2_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/open").c_str());

        this->srvMV2_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv2/close", std::bind(&Node::MV2_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/close").c_str());

        this->srvMV2_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/mv2/get_state", std::bind(&Node::MV2_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2/get_state").c_str());

        this->srvPV_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/pv/open", std::bind(&Node::PV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/open").c_str());

        this->srvPV_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/pv/close", std::bind(&Node::PV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/close").c_str());

        this->srvPV_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/pv/get_state", std::bind(&Node::PV_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv/get_state").c_str());

        this->srvBV_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/bv/open", std::bind(&Node::BV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/open").c_str());

        this->srvBV_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/bv/close", std::bind(&Node::BV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/close").c_str());

        this->srvBV_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/bv/get_state", std::bind(&Node::BV_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/get_state").c_str());

        this->srvFire = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/fire", std::bind(&Node::Fire, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/fire").c_str());

        try {
            rdata::iface::createLogger(this->clCreateLogger->get_service_name(),
                                       this->get_node_base_interface(),
                                       this->clCreateLogger,
                                       "recu/logger");
        } catch (const rutil::except::service_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure node\nWhat: %s", e.what());

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());
        // Cannot activate subscribers
        // REFERENCE: https://github.com/ros2/demos/issues/488

        this->logger->on_activate();
        this->read_timer->reset();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());
        // Cannot deactivate subscribers
        // REFERENCE: https://github.com/ros2/demos/issues/488

        this->read_timer->cancel();
        this->logger->on_deactivate();

        this->stateMV1 = Unknown;
        this->stateMV2 = Unknown;
        this->statePV = Unknown;
        this->stateBV = Unknown;

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->serialClose();
        this->deleteAllPointers();

        try {
            rdata::iface::removeLogger(this->clRemoveLogger->get_service_name(),
                                       this->get_node_base_interface(),
                                       this->clRemoveLogger,
                                       "recu/logger");
        } catch (const rutil::except::service_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to cleanup node\nWhat: %s", e.what());

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->serialClose();
        this->deleteAllPointers();

        this->stateMV1 = Unknown;
        this->stateMV2 = Unknown;
        this->statePV = Unknown;
        this->stateBV = Unknown;

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {
        this->srvMV1_Open.reset();
        this->srvMV1_Close.reset();
        this->srvMV1_GetState.reset();
        this->srvMV2_Open.reset();
        this->srvMV2_Close.reset();
        this->srvMV2_GetState.reset();
        this->srvPV_Open.reset();
        this->srvPV_Close.reset();
        this->srvPV_GetState.reset();
        this->srvBV_Open.reset();
        this->srvBV_Close.reset();
        this->srvBV_GetState.reset();
    }

    void Node::serialRead() {
        // Temp buffer for current read
        char temp_buf[256];
        int n = read(serial_port, &temp_buf, sizeof(temp_buf));

        if (n < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read serial port");
        }

        // For bytes read, append to past reads
        // If newline is read, handle complete message
        for (int i = 0; i < n; i++) {
            if (temp_buf[i] == '\n') {
                try {
                    this->serialHandleMsg();
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Failed JSON deserialization");
                }
            } else {
                this->read_buf[this->read_buf_l] = temp_buf[i];
                this->read_buf_l++;
            }
        }
    }

    void Node::serialHandleMsg() {
        std::string str(this->read_buf, this->read_buf_l);
        this->read_buf_l = 0;

        json j = json::parse(str);

        JsonData data{
            j["mv1"].get<bool>(),
            j["mv2"].get<bool>(),
            j["pv"].get<bool>(),
            j["bv"].get<bool>(),
            j["LC0"].get<float>(),
            j["tPS0"].get<float>(),
            j["tPS1"].get<float>(),
            j["ccPS0"].get<float>(),
            j["ccPS1"].get<float>(),
            j["T0"].get<float>(),
            j["T1"].get<float>(),
            j["T2"].get<float>(),
        };

        if (data.stateMV1) {
            this->stateMV1 = Open;
        } else {
            this->stateMV1 = Closed;
        }

        if (data.stateMV2) {
            this->stateMV2 = Open;
        } else {
            this->stateMV2 = Closed;
        }

        if (data.statePV) {
            this->statePV = Open;
        } else {
            this->statePV = Closed;
        }

        if (data.stateBV) {
            this->stateBV = Open;
        } else {
            this->stateBV = Closed;
        }

        // Load cell
        auto message = rdata_msgs::msg::LogF64();
        message.measurment = "force";
        message.sensor = "load_cell";
        message.value = data.loadCell;
        this->logger->publish(message);

        // Pressure sensor 1 (tank)
        message = rdata_msgs::msg::LogF64();
        message.measurment = "pressure";
        message.sensor = "ps_tank_1";
        message.value = data.pressureTank1;
        this->logger->publish(message);

        // Pressure sensor 2 (tank)
        message = rdata_msgs::msg::LogF64();
        message.measurment = "pressure";
        message.sensor = "ps_tank_2";
        message.value = data.pressureTank2;
        this->logger->publish(message);

        // Pressure sensor 1 (chamber)
        message = rdata_msgs::msg::LogF64();
        message.measurment = "pressure";
        message.sensor = "ps_chamber_1";
        message.value = data.pressureChamber1;
        this->logger->publish(message);

        // Pressure sensor 2 (chamber)
        message = rdata_msgs::msg::LogF64();
        message.measurment = "pressure";
        message.sensor = "ps_chamber_2";
        message.value = data.pressureChamber2;
        this->logger->publish(message);

        // Thermocouple 1
        message = rdata_msgs::msg::LogF64();
        message.measurment = "temp";
        message.sensor = "thermocouple_1";
        message.value = data.tempThermocouple1;
        this->logger->publish(message);

        // Thermocouple 2
        message = rdata_msgs::msg::LogF64();
        message.measurment = "temp";
        message.sensor = "thermocouple_2";
        message.value = data.tempThermocouple2;
        this->logger->publish(message);

        // Thermocouple 3
        message = rdata_msgs::msg::LogF64();
        message.measurment = "temp";
        message.sensor = "thermocouple_3";
        message.value = data.tempThermocouple3;
        this->logger->publish(message);
    }

    void Node::serialWrite(EcuActions action) { write(this->serial_port, &action, sizeof(action)); }

    void Node::serialClose() {
        this->read_timer->cancel();
        close(this->serial_port);
    }

    recu_msgs::srv::GetValveState::Response Node::createValveStateResponse(ValveState valveState) {
        recu_msgs::srv::GetValveState::Response response;

        response.current_state.id = valveState;

        switch (valveState) {
        case Closed:
            response.current_state.label = "closed";
            break;
        case Open:
            response.current_state.label = "open";
            break;
        case Unknown:
            response.current_state.label = "unkown";
            break;
        }

        return response;
    }

    void Node::MV1_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::MV1_Open);
    }

    void Node::MV1_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                         std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::MV1_Close);
    }

    void Node::MV1_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                            std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        *response = createValveStateResponse(this->stateMV1);
    }

    void Node::MV2_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::MV2_Open);
    }

    void Node::MV2_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                         std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::MV2_Close);
    }

    void Node::MV2_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                            std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        *response = createValveStateResponse(this->stateMV2);
    }

    void Node::PV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::PV_Open);
    }

    void Node::PV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::PV_Close);
    }

    void Node::PV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                           std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        *response = createValveStateResponse(this->statePV);
    }

    void Node::BV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                       std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::BV_Open);
    }

    void Node::BV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::BV_Close);
    }

    void Node::BV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                           std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        *response = createValveStateResponse(this->stateBV);
    }

    void Node::Fire(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                    std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        RCLCPP_INFO(this->get_logger(), "FIRING!");
        this->serialWrite(EcuActions::Fire);
    }
} // namespace recu

#include <node.hpp>

namespace rtty {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("recu") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->read_timer = this->create_wall_timer(1ms, std::bind(&Node::serialRead, this));
        this->read_timer->cancel();

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
        tty.c_cc[VTIME] = 0; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 1;
        // Baud Rate
        cfsetspeed(&tty, B9600);

        // Save tty settings
        if (tcsetattr(this->serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return LifecycleCallbackReturn::FAILURE;
        }

        this->srvMV1_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv1_open", std::bind(&Node::MV1_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1_open").c_str());

        this->srvMV1_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv1_close", std::bind(&Node::MV1_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv1_close").c_str());

        this->srvMV2_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv2_open", std::bind(&Node::MV2_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2_open").c_str());

        this->srvMV2_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/mv2_close", std::bind(&Node::MV2_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/mv2_close").c_str());

        this->srvPV_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/pv_open", std::bind(&Node::PV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv_open").c_str());

        this->srvPV_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/pv_close", std::bind(&Node::PV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/pv_close").c_str());

        this->srvESV_Open = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/esv_open", std::bind(&Node::ESV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/esv_open").c_str());

        this->srvESV_Close = this->create_service<recu_msgs::srv::ArduinoAction>(
            "recu/esv_close", std::bind(&Node::ESV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/esv_close").c_str());

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());
        // Cannot activate subscribers
        // REFERENCE: https://github.com/ros2/demos/issues/488

        this->read_timer->reset();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());
        // Cannot deactivate subscribers
        // REFERENCE: https://github.com/ros2/demos/issues/488

        this->read_timer->cancel();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->serialClose();
        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->serialClose();
        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {
        this->srvMV1_Open.reset();
        this->srvMV1_Close.reset();
        this->srvMV2_Open.reset();
        this->srvMV2_Close.reset();
        this->srvPV_Open.reset();
        this->srvPV_Close.reset();
        this->srvESV_Open.reset();
        this->srvESV_Close.reset();
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
                this->serialHandleMsg();
            }

            this->read_buf[read_buf_l] = temp_buf[i];
            this->read_buf_l++;
        }
    }

    void Node::serialHandleMsg() {
        std::string out(this->read_buf, this->read_buf_l);
        std::cout << out << "\n";
        this->read_buf_l = 0;
    }

    void Node::serialWrite(EcuActions action) {
        RCLCPP_INFO(this->get_logger(), "Sending action '%d' to serial port", action);
        write(this->serial_port, &action, sizeof(action));
    }

    void Node::serialClose() {
        this->read_timer->cancel();
        close(this->serial_port);
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

    void Node::ESV_Open(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::ESV_Open);
    }

    void Node::ESV_Close(const std::shared_ptr<recu_msgs::srv::ArduinoAction::Request> request,
                         std::shared_ptr<recu_msgs::srv::ArduinoAction::Response> response) {
        (void)request;
        (void)response;
        this->serialWrite(EcuActions::ESV_Close);
    }
} // namespace rtty

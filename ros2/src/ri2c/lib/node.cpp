#include <node.hpp>

namespace ri2c {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("ri2c") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->declare_parameter<std::string>("config_path", "/home/ros/ri2c/config.toml");
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        // try {
        //     toml::table toml = toml::parse_file(this->get_parameter("config_path").as_string());
        //     readConfig(toml);
        // } catch (rgpio::except::config_parse_error &e) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
        //     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        // }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        int file = open("/dev/i2c-1", O_RDWR);

        if (file < 0) {
            RCLCPP_INFO(this->get_logger(), "failed to open file");
            // return 1;
        }

        int addr = 0x48;

        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            RCLCPP_INFO(this->get_logger(), "failed to set i2c slave");
            // return 1;
        }

        i2c_smbus_write_byte(file, 1);
        sleep(1);

        __u8 reg = 0x00; /* Device register to access */
        __u8 res;

        /* Using SMBus commands */

        res = i2c_smbus_read_byte_data(file, reg);
        if (res < 0) {
            RCLCPP_INFO(this->get_logger(), "failed to read");
            // return 1;
        }

        uint f;
        std::memcpy(&f, &res, sizeof(f));

        RCLCPP_INFO(this->get_logger(), "read: %d", f);

        close(file);
        // return 0;

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {}
} // namespace ri2c

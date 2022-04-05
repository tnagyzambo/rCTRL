#include <node.hpp>
#include <sstream>

namespace rgpio_example {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("rgpio_example") {}

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    LifecycleCallbackReturn Node::on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            toml::table toml = toml::parse_file("/workspaces/rCTRL/ros2/src/rgpio_example/gpio.toml");
            readConfig(toml);
        } catch (rgpio::except::config_parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
            return LifecycleCallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::active().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::deactivating().c_str());

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::cleaningUp().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::unconfigured().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::shuttingDown().c_str());

        this->deleteAllPointers();

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::finalized().c_str());

        return LifecycleCallbackReturn::SUCCESS;
    }

    void Node::deleteAllPointers() {}

    // Parse .toml and create all commands
    void Node::readConfig(toml::table toml) {
        auto tomlView = toml::node_view(toml);

        auto gpioView = rutil::toml::viewOfTable(tomlView, "sense");

        this->sense = std::make_unique<rgpio::Input>(rgpio::Input((rclcpp::Node *)this, gpioView, "sense"));
    }
} // namespace rgpio_example

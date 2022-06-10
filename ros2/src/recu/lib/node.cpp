#include <node.hpp>
#include <recu_msgs/srv/detail/get_valve_state__struct.hpp>
#include <rgpio/output.hpp>
#include <sstream>

namespace recu {
    Node::Node() : rclcpp_lifecycle::LifecycleNode("recu") {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::constructing().c_str());

        this->declare_parameter<std::string>("config_path", "/home/ros/recu/config.toml");
    }

    Node::~Node() { RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::destructing().c_str()); }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::configuring().c_str());

        try {
            toml::table toml = toml::parse_file(this->get_parameter("config_path").as_string());
            readConfig(toml);
        } catch (rgpio::except::config_parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure!\nError: %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        this->srvBV_Open = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/bv/open", std::bind(&Node::BV_Open, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/open").c_str());

        this->srvBV_Close = this->create_service<recu_msgs::srv::ValveAction>(
            "recu/bv/close", std::bind(&Node::BV_Close, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/close").c_str());

        this->srvBV_GetState = this->create_service<recu_msgs::srv::GetValveState>(
            "recu/bv/get_state", std::bind(&Node::BV_GetState, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::srv::created("recu/bv/get_state").c_str());

        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::state::inactive().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(
        const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "%s", rutil::fmt::transition::activating().c_str());

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

    void Node::deleteAllPointers() { this->valveBV.reset(); }

    // Parse .toml and create all commands
    void Node::readConfig(toml::table toml) {
        auto tomlView = toml::node_view(toml);

        auto gpioView = rutil::toml::viewOfTable(tomlView, "gpio");

        this->valveBV = std::make_unique<rgpio::Output>(rgpio::Output((rclcpp::Node *)this, gpioView, "valveBV"));
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
        default:
            response.current_state.label = "unkown";
            break;
        }

        return response;
    }

    void Node::BV_Open(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                       std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        this->valveBV->write(rgpio::gpio::line_level::HIGH);
    }

    void Node::BV_Close(const std::shared_ptr<recu_msgs::srv::ValveAction::Request> request,
                        std::shared_ptr<recu_msgs::srv::ValveAction::Response> response) {
        (void)request;
        (void)response;
        this->valveBV->write(rgpio::gpio::line_level::LOW);
    }

    void Node::BV_GetState(const std::shared_ptr<recu_msgs::srv::GetValveState::Request> request,
                           std::shared_ptr<recu_msgs::srv::GetValveState::Response> response) {
        (void)request;
        *response = createValveStateResponse((ValveState)rgpio::gpio::line_level::toInt(this->valveBV->getState()));
    }
} // namespace recu

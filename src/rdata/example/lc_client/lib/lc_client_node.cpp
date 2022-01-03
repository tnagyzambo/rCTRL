#include <lc_client_node.hpp>

rdata::lc_client::Node::Node(const char *nodeName) : rclcpp::Node(nodeName)
{
    this->rdata = std::make_unique<rctrl::lc::Client>(this->get_node_base_interface(),
                                                      rdata::iface::nodeName.data(),
                                                      this->create_client<lifecycle_msgs::srv::GetState>(rdata::iface::srv_get_state.data()),
                                                      this->create_client<lifecycle_msgs::srv::ChangeState>(rdata::iface::srv_change_state.data()));

    this->vF64 = std::make_unique<rctrl::lc::Client>(this->get_node_base_interface(),
                                                     rdata::iface::nodeName.data(),
                                                     this->create_client<lifecycle_msgs::srv::GetState>(rdata::iface::srv_get_state.data()),
                                                     this->create_client<lifecycle_msgs::srv::ChangeState>(rdata::iface::srv_change_state.data()));

    this->timer = this->create_wall_timer(50ms, std::bind(&rdata::lc_client::Node::run, this));

    this->subRosout = this->create_subscription<rcl_interfaces::msg::Log>("rosout", 10, std::bind(&rdata::lc_client::Node::callbackSubRosout, this, std::placeholders::_1));
}

rdata::lc_client::Node::~Node()
{
}

void rdata::lc_client::Node::run()
{
}

void rdata::lc_client::Node::callbackSubRosout(const rcl_interfaces::msg::Log::SharedPtr msg) const
{
    (void)msg;
}
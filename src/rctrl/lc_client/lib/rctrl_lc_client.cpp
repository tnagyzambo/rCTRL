#include <rctrl_lc_client.hpp>

rctrl::lc::Client::Client(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr callingNode,
                          const char *serviceNodeName,
                          std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> clGetState,
                          std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> clChangeState)
    : callingNode(callingNode),
      serviceNodeName(serviceNodeName),
      clGetState(clGetState),
      clChangeState(clChangeState)
{
}

rctrl::lc::Client::~Client()
{
}

unsigned int rctrl::lc::Client::getState(std::chrono::seconds timeOut = 3s)
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!this->clGetState->wait_for_service(timeOut))
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->callingNode->get_name()), "Service %s is not available.", clGetState->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = clGetState->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = waitForResult(future_result, timeOut);

    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->callingNode->get_name()), "Server time out while getting current state for node %s", this->serviceNodeName);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get())
    {
        RCLCPP_INFO(rclcpp::get_logger(this->callingNode->get_name()), "Node %s has current state %s.", this->serviceNodeName, future_result.get()->current_state.label.c_str());
        return future_result.get()->current_state.id;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->callingNode->get_name()), "Failed to get current state for node %s", this->serviceNodeName);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool rctrl::lc::Client::changeState(std::uint8_t transition, std::chrono::seconds timeOut = 3s)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!clChangeState->wait_for_service(timeOut))
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->callingNode->get_name()), "Service %s is not available.", clChangeState->get_service_name());
        return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = clChangeState->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = waitForResult(future_result, timeOut);

    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->callingNode->get_name()), "Server time out while getting current state for node %s", this->serviceNodeName);
        return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success)
    {
        RCLCPP_INFO(rclcpp::get_logger(this->callingNode->get_name()), "Transition %d successfully triggered.", static_cast<int>(transition));
        return true;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger(this->callingNode->get_name()), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
        return false;
    }
}
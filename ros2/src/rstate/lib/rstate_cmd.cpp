#include <rstate_cmd.hpp>

namespace rstate {
    template <>
    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> Cmd<lifecycle_msgs::srv::ChangeState>::createRequest(
        toml::table toml) {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = (uintptr_t)toml["request"]["transition"]["id"].as_integer();
        request->transition.label = (const char *)toml["request"]["transition"]["label"].as_string();

        return request;
    }
} // namespace rstate

#include "rstate_util.hpp"
#include <rstate_cmd.hpp>

namespace rstate {
    template <>
    void Cmd<lifecycle_msgs::srv::ChangeState>::createRequests(toml::table toml) {
        // Main request
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        request->transition.id = util::getTomlEntryByKey<uint>(*toml["request"]["transition"].as_table(), "id");
        request->transition.label =
            util::getTomlEntryByKey<std::string>(*toml["request"]["transition"].as_table(), "label");

        this->request = request;

        // Main response
        auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        response->success = util::getTomlEntryByKey<bool>(*toml["response"].as_table(), "success");

        this->response = response;

        // Cancel request
        auto requestCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        requestCancel->transition.id =
            util::getTomlEntryByKey<uint>(*toml["cancel"]["request"]["transition"].as_table(), "id");
        requestCancel->transition.label =
            util::getTomlEntryByKey<std::string>(*toml["cancel"]["request"]["transition"].as_table(), "label");

        this->requestCancel = requestCancel;

        // Cancel response
        auto responseCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        responseCancel->success = util::getTomlEntryByKey<bool>(*toml["cancel"]["response"].as_table(), "success");

        this->responseCancel = responseCancel;
    }

    template <>
    void Cmd<lifecycle_msgs::srv::ChangeState>::compareResponse(
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response,
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> expectedResponse) {
        if (response->success != expectedResponse->success) {
            std::stringstream error;

            error << "Received unexpected response from service '" << this->client->get_service_name() << "'\n";
            error << "Recieved: success = " << response->success << "\n";
            error << "Expected: success = " << expectedResponse->success << "\n";

            throw(except::cmd_service_eror(error.str()));
        }
    }
} // namespace rstate

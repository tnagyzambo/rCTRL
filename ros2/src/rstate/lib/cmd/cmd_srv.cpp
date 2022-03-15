#include <cmd_srv.hpp>

namespace rstate {
    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::createRequest(toml::table toml) {
        // Main request
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        request->transition.id =
            util::getTomlEntryByKey<uint>(util::getTomlTableBySections(toml, "request", "transition"), "id");
        request->transition.label =
            util::getTomlEntryByKey<std::string>(util::getTomlTableBySections(toml, "request", "transition"), "label");

        this->request = request;

        // Main response
        auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        response->success = util::getTomlEntryByKey<bool>(util::getTomlTableBySections(toml, "response"), "success");

        this->response = response;
    }

    template <>
    void CmdServiceCancelable<lifecycle_msgs::srv::ChangeState>::createRequestCancel(toml::table toml) {
        // Cancel request
        auto requestCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        requestCancel->transition.id =
            util::getTomlEntryByKey<uint>(util::getTomlTableBySections(toml, "cancel", "request", "transition"), "id");
        requestCancel->transition.label = util::getTomlEntryByKey<std::string>(
            util::getTomlTableBySections(toml, "cancel", "request", "transition"), "label");

        this->requestCancel = requestCancel;

        // Cancel response
        auto responseCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        responseCancel->success =
            util::getTomlEntryByKey<bool>(util::getTomlTableBySections(toml, "cancel", "response"), "success");

        this->responseCancel = responseCancel;
    }

    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::compareResponse(
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

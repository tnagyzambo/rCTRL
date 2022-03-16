#include <service.hpp>

namespace rstate {
    template <>
    void CmdService<lifecycle_msgs::srv::ChangeState>::createRequest(toml::node_view<toml::node> toml) {
        // Main request
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        auto requestToml = util::toml::viewOfTable(toml, "request");
        auto transitionToml = util::toml::viewOfTable(requestToml, "transition");
        request->transition.id = util::getTomlEntryByKey<uint>(*transitionToml.as_table(), "id");
        request->transition.label = util::getTomlEntryByKey<std::string>(*transitionToml.as_table(), "label");

        this->request = request;

        // Main response
        auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        auto responseToml = util::toml::viewOfTable(toml, "response");
        response->success = util::getTomlEntryByKey<bool>(*responseToml.as_table(), "success");

        this->response = response;
    }

    template <>
    void CmdServiceCancelable<lifecycle_msgs::srv::ChangeState>::createRequestCancel(toml::node_view<toml::node> toml) {
        // Cancel request
        auto requestCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        auto cancelToml = util::toml::viewOfTable(toml, "cancel");

        auto requestToml = util::toml::viewOfTable(cancelToml, "request");
        auto transitionToml = util::toml::viewOfTable(requestToml, "transition");
        requestCancel->transition.id = util::getTomlEntryByKey<uint>(*transitionToml.as_table(), "id");
        requestCancel->transition.label = util::getTomlEntryByKey<std::string>(*transitionToml.as_table(), "label");

        this->requestCancel = requestCancel;

        // Cancel response
        auto responseCancel = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();

        auto responseToml = util::toml::viewOfTable(cancelToml, "response");
        responseCancel->success = util::getTomlEntryByKey<bool>(*responseToml.as_table(), "success");

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

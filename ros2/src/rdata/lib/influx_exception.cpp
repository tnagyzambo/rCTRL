#include <influx_exception.hpp>

namespace influx::except {
    PostReq::PostReq(std::string response) : std::runtime_error(buildMessage(response)) {}

    std::string PostReq::buildMessage(std::string response) {
        std::stringstream message;

        message << "\033[1;31mRocketDATA has failed a POST request!\033[0m Response: ";
        message << response;
        message << std::endl;

        return message.str();
    }

    Curl::Curl(std::string response) : std::runtime_error(buildMessage(response)) {}

    std::string Curl::buildMessage(std::string response) {
        std::stringstream message;

        message << "\033[1;31mRocketDATA has encountered a critical CURL error! \033[0m";
        message << response;
        message << std::endl;

        return message.str();
    }
} // namespace influx::except

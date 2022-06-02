#include <rdata/except.hpp>

namespace rdata::except {
    PostReq::PostReq(std::string response) : std::runtime_error(buildMessage(response)) {}

    std::string PostReq::buildMessage(std::string response) {
        std::string message = fmt::format("rDATA has failed a POST request!\n{}", response);

        return message;
    }

    Curl::Curl(std::string response) : std::runtime_error(buildMessage(response)) {}

    std::string Curl::buildMessage(std::string response) {
        std::string message = fmt::format("rDATA has encountered a critical CURL error!\n{}", response);

        return message;
    }
} // namespace rdata::except

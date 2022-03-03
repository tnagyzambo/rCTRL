#include <influx_exception.hpp>

influx::except::PostReq::PostReq(std::string response) : std::runtime_error(buildMessage(response))
{
}

std::string influx::except::PostReq::buildMessage(std::string response)
{
    std::stringstream message;

    message << "\033[1;31mRocketDATA has failed a POST request!\033[0m Response: ";
    message << response;
    message << std::endl;

    return message.str();
}

influx::except::Curl::Curl(std::string response) : std::runtime_error(buildMessage(response))
{
}

std::string influx::except::Curl::buildMessage(std::string response)
{
    std::stringstream message;

    message << "\033[1;31mRocketDATA has encountered a critical CURL error! \033[0m";
    message << response;
    message << std::endl;

    return message.str();
}

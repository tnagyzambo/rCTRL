#include <InfluxClientException.hpp>

influxclient::PostRequestException::PostRequestException(std::string response) : std::runtime_error(buildMessage(response))
{
}

std::string influxclient::PostRequestException::buildMessage(std::string response)
{
    std::stringstream message;

    message << "\033[1;31mRocketDATA has failed a POST request!\033[0m Response: ";
    message << response;
    message << std::endl;

    return message.str();
}

influxclient::CurlException::CurlException(std::string response) : std::runtime_error(buildMessage(response))
{
}

std::string influxclient::CurlException::buildMessage(std::string response)
{
    std::stringstream message;

    message << "\033[1;31mRocketDATA has encountered a critical CURL error! \033[0m";
    message << response;
    message << std::endl;

    return message.str();
}

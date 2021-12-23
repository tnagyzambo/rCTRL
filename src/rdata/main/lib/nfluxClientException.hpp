#pragma once

#include <string>
#include <sstream>
#include <stdexcept>

namespace influxclient
{
    class PostRequestException : public std::runtime_error
    {
    public:
        PostRequestException(std::string);

    private:
        std::string buildMessage(std::string);
    };

    class CurlException : public std::runtime_error
    {
    public:
        CurlException(std::string);

    private:
        std::string buildMessage(std::string);
    };
}
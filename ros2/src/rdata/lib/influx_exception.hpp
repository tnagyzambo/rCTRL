#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

namespace influx::except {
    class PostReq : public std::runtime_error {
    public:
        PostReq(std::string);

    private:
        std::string buildMessage(std::string);
    };

    class Curl : public std::runtime_error {
    public:
        Curl(std::string);

    private:
        std::string buildMessage(std::string);
    };
} // namespace influx::except
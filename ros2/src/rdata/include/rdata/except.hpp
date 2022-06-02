#pragma once

#include <fmt/format.h>
#include <stdexcept>
#include <string>

namespace rdata::except {
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
} // namespace rdata::except

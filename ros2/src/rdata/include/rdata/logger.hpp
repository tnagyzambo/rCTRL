#pragma once

#include <chrono>
#include <curl/curl.h>
#include <fmt/format.h>
#include <iostream>
#include <rdata/except.hpp>
#include <rutil/toml.hpp>
#include <string>
#include <sys/time.h>
#include <toml++/toml.h>

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;

namespace rdata {
    // Structure to hold the basic info required to make post requests to InfluxDB
    struct Credentials {
        std::string path;
        std::string user;
        std::string password;
        std::string token;
        std::string org;
        std::string bucket;
        std::string retention;
    };

    class Logger {
    public:
        Logger(std::string);
        ~Logger();

        void log(std::string);

        void writeToInflux();

    private:
        static const std::string name;
        Credentials credentials;
        std::string urlWrite;
        std::string authorization;

        std::string postBody;

        long postRequest(std::string &, struct curl_slist *, std::string &, std::string &);
        static size_t writeCallback(char *, size_t, size_t, void *);

        Credentials getCredentials(std::string);

        std::string constructInfluxURL();
        std::string constructInfluxAuthorization();
        std::string constructInfluxPostBody(std::string, std::string, std::string);
    };
} // namespace rdata

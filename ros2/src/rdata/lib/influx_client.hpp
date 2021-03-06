#pragma once

#include <curl/curl.h>
#include <influx_exception.hpp>
#include <iostream>
#include <string>
#include <toml++/toml.h>

#define CREDENTIALS_FILE "/home/ros/rdata/influx/credentials.toml"

namespace influx {
    // Structure to hold the basic info required to make post requests to InfluxDB
    struct Credentials {
        std::string user;
        std::string password;
        std::string token;
        std::string org;
        std::string bucket;
        std::string retention;
    };

    class Client {
    public:
        Credentials credentials;
        std::string urlWrite;
        std::string authorization;

        Client();
        ~Client();

        void printCredentials();

        // Templated function to allow writing all supported types to InfluxDB
        // Templated functions must implemented in the header or a separate .tpp file appended to the bottom of the header
        template <typename T> void writeToInflux(std::string, std::string, T);

    private:
        long postRequest(std::string &, struct curl_slist *, std::string &, std::string &);
        static size_t writeCallback(char *, size_t, size_t, void *);

        Credentials getCredentials();
        toml::table getToml(std::string);
        std::string getTomlEntryBySectionKey(toml::table, std::string, std::string);

        std::string constructInfluxURL();
        std::string constructInfluxAuthorization();

        std::string constructInfluxValueString(bool);
        std::string constructInfluxValueString(double);
        std::string constructInfluxValueString(std::string);
        std::string constructInfluxValueString(int64_t);
        std::string constructInfluxValueString(uint64_t);

        std::string constructInfluxPostBody(std::string, std::string, std::string);
    };

// Implementation of templated functions
#include "influx_client.tpp"
} // namespace influx
#pragma once

#include <string>
#include <iostream>
#include <curl/curl.h>
#include <toml++/toml.h>

#define CREDENTIALS_FILE "/rocketDATA/influx/credentials.toml"

struct Credentials {
    std::string user;
    std::string password;
    std::string token;
    std::string org;
    std::string bucket;
    std::string retention;
};

class InfluxClient
{
    public:
        Credentials credentials;
        std::string url;
        std::string authorization;
        struct curl_slist* headers = NULL;

        InfluxClient();
        ~InfluxClient();

        void printCredentials();

        template <typename T>
        void writeToInflux(std::string, std::string, T);

    private:
        CURL *curl;
        std::string curlReadBuffer;
        CURLcode res;

        static size_t writeCallback(char *, size_t, size_t, void*);
        
        Credentials getCredentials();
        std::string promptForBucket(std::string);
        toml::table getToml(std::string);
        std::string getTomlEntryBySectionKey(toml::table, std::string, std::string);

        std::string constructInfluxURL();
        std::string constructInfluxAuthorization();

        template <typename T>
        std::string constructInfluxValueString(T);
};

#include "InfluxClient.tpp"

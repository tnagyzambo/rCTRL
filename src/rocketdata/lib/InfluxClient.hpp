#pragma once

#include <string>
#include <iostream>
#include <toml++/toml.h>

#define CREDENTIALS_FILE "/rocketDATA/influx/credentials.toml"

struct Info {
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
        InfluxClient();

        Info info;

        void printCredentials();

    private:
        toml::table credentials;

        void getCredentials();
        std::string promptForBucket(std::string);
        toml::table getToml(std::string);
        std::string getTomlEntryBySectionKey(toml::table, std::string, std::string);
};
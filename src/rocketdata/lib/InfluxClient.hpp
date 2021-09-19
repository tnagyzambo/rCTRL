#pragma once
#include <string>
#define TOKEN_FILE "/rocketDATA/influx/credentials.toml"

struct Info {
    std::string token;
    std::string user;
    std::string password;
    std::string org;
    std::string bucket;
    std::string retention;
};

class InfluxClient
{
    public:
        InfluxClient();
        Info info;

    private:
        void getCredentials();
        void getBucketName();
};
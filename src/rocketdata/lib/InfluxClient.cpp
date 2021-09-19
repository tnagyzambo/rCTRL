#include "InfluxClient.hpp"
#include <iostream>
#include <fstream>

InfluxClient::InfluxClient() {
    this->getCredentials();
    this->info.bucket = "none";

}

void InfluxClient::getCredentials() {
    std::ifstream ifs (TOKEN_FILE, std::ifstream::in);

    char c = ifs.get();

    while (ifs.good()) {
        std::cout << c;
        c = ifs.get();
    }

    ifs.close();

    return;
}
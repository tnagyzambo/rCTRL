#include "InfluxClient.hpp"

InfluxClient::InfluxClient() {
    this->getCredentials();
}

void InfluxClient::getCredentials() {
    toml::table tbl = getToml(CREDENTIALS_FILE);

    this->info.user = getTomlEntryBySectionKey(tbl, "credentials", "user");
    this->info.password = getTomlEntryBySectionKey(tbl, "credentials", "password");
    this->info.token = getTomlEntryBySectionKey(tbl, "credentials", "token");
    this->info.org = getTomlEntryBySectionKey(tbl, "credentials", "org");
    this->info.retention = getTomlEntryBySectionKey(tbl, "default-params", "retention");

    std::string bucket = getTomlEntryBySectionKey(tbl, "default-params", "bucket");
    this->info.bucket = promptForBucket(bucket);

    return;
}

void InfluxClient::printCredentials() {
    std::cout << "\033[1mUser: \033[0m" << this->info.user << "\n";
    std::cout << "\033[1mPassword: \033[0m" << this->info.password << "\n";
    std::cout << "\033[1mToken: \033[0m" << this->info.token << "\n";
    std::cout << "\033[1mOrg: \033[0m" << this->info.org << "\n";
    std::cout << "\033[1mBucket: \033[0m" << this->info.bucket << "\n";
    std::cout << "\033[1mRetention: \033[0m" << this->info.retention << "\n";

    return;
}

std::string InfluxClient::promptForBucket(std::string bucket) {
    std::string input;

    std::cout << "\033[1;36mEnter bucket for current data logging session.\033[0m" << std::endl;
    std::cout << "\033[1mLeave empty to use default bucket \"" << bucket << "\".\033[0m" << std::endl;
    std::cout << "\033[1mBucket: \033[0m";
    std::getline(std::cin, input);
    std::cout << std::endl;

    if (input != "") {
        bucket = input;
    }

    return bucket;
}

toml::table InfluxClient::getToml(std::string path) {
    toml::table tbl;

    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error& e) {
        std::cout << "\033[1;31mCANNOT PARSE CREDENTIALS FILE!\033[0m" << std::endl;
        std::cout << "\033[1mError: \033[0m" << e << "\n";
        
        exit(EXIT_FAILURE);
    }

    return tbl;
}

std::string InfluxClient::getTomlEntryBySectionKey(toml::table tbl, std::string section, std::string key) {
    std::optional<std::string> entry = tbl[section][key].value<std::string>();

    try {
        entry.value();
    } catch (const std::bad_optional_access& e) {
        std::cout << "\033[1;31mNO \"" << key << "\" FOUND IN CREDENTIALS FILE!\033[0m" << std::endl;
        std::cout << "\033[1mFile Path: \033[0m" << CREDENTIALS_FILE << std::endl;
        std::cout << "\033[1mSection: \033[0m" << section << std::endl;
        std::cout << "\033[1mKey: \033[0m" << key << std::endl;
        std::cout << "\033[1mError: \033[0m" << e.what() << std::endl;
        std::cout << std::endl;

        exit(EXIT_FAILURE);
    }
    
    return entry.value();
}

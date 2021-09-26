#include <InfluxClient.hpp>

InfluxClient::InfluxClient() {
    this->credentials = getCredentials();
    this->url = constructInfluxURL();
    this->authorization = constructInfluxAuthorization();
    this->headers = curl_slist_append(this->headers, this->authorization.c_str());

    curl_global_init(CURL_GLOBAL_ALL);
    this->curl = curl_easy_init();
    
    curl_easy_setopt(this->curl, CURLOPT_URL, "http://localhost:8086/api/v2/write?org=ros&bucket=default&precision=ns");
    curl_easy_setopt(this->curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(this->curl, CURLOPT_POST, 1L);
    curl_easy_setopt(this->curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(this->curl, CURLOPT_WRITEFUNCTION, InfluxClient::writeCallback);
    curl_easy_setopt(this->curl, CURLOPT_WRITEDATA, &this->curlReadBuffer);
}

InfluxClient::~InfluxClient() {
    curl_global_cleanup();
}

size_t InfluxClient::writeCallback(char *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);

    return size * nmemb;
}

Credentials InfluxClient::getCredentials() {
    Credentials credentials;

    toml::table tbl = getToml(CREDENTIALS_FILE);

    credentials.user = getTomlEntryBySectionKey(tbl, "credentials", "user");
    credentials.password = getTomlEntryBySectionKey(tbl, "credentials", "password");
    credentials.token = getTomlEntryBySectionKey(tbl, "credentials", "token");
    credentials.org = getTomlEntryBySectionKey(tbl, "credentials", "org");
    credentials.retention = getTomlEntryBySectionKey(tbl, "default-params", "retention");

    std::string bucket = getTomlEntryBySectionKey(tbl, "default-params", "bucket");
    credentials.bucket = promptForBucket(bucket);

    return credentials;
}

void InfluxClient::printCredentials() {
    std::cout << std::endl;
    std::cout << "\033[1mUser: \033[0m" << this->credentials.user << std::endl;
    std::cout << "\033[1mPassword: \033[0m" << this->credentials.password << std::endl;
    std::cout << "\033[1mToken: \033[0m" << this->credentials.token << std::endl;
    std::cout << "\033[1mOrg: \033[0m" << this->credentials.org << std::endl;
    std::cout << "\033[1mBucket: \033[0m" << this->credentials.bucket << std::endl;
    std::cout << "\033[1mRetention: \033[0m" << this->credentials.retention << std::endl;

    return;
}

std::string InfluxClient::promptForBucket(std::string bucket) {
    std::string input;

    std::cout << std::endl;
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
        std::stringstream error;

        error << std::endl;
        error << "\033[1;31mCANNOT PARSE CREDENTIALS FILE!\033[0m" << std::endl;
        error << "\033[1mError: \033[0m" << e << std::endl;
        
        throw std::runtime_error(error.str());
    }

    return tbl;
}

std::string InfluxClient::getTomlEntryBySectionKey(toml::table tbl, std::string section, std::string key) {
    std::optional<std::string> entry = tbl[section][key].value<std::string>();

    try {
        entry.value();
    } catch (const std::bad_optional_access& e) {
        std::stringstream error;

        error << std::endl;
        error << "\033[1;31mNO \"" << key << "\" FOUND IN CREDENTIALS FILE!\033[0m" << std::endl;
        error << "\033[1mFile Path: \033[0m" << CREDENTIALS_FILE << std::endl;
        error << "\033[1mSection: \033[0m" << section << std::endl;
        error << "\033[1mKey: \033[0m" << key << std::endl;
        error << "\033[1mError: \033[0m" << e.what() << std::endl;
        error << std::endl;

        throw std::runtime_error(error.str());
    }
    
    return entry.value();
}

std::string InfluxClient::constructInfluxURL() {
    std::string url = "http://localhost:8086/api/v2/write?org=";
    url.append(this->credentials.org);
    url.append("&bucket=");
    url.append(this->credentials.bucket);
    url.append("&precision=ns");

    return url;
}

std::string InfluxClient::constructInfluxAuthorization() {
    std::string authorization = "Authorization: Token ";
    authorization.append(this->credentials.token);

    return authorization;
}

template<>
std::string InfluxClient::constructInfluxValueString(float value) {
    std::string influxValueString = std::to_string(value);

    return influxValueString;
}

template<>
std::string InfluxClient::constructInfluxValueString(int value) {
    std::string influxValueString = std::to_string(value);
    influxValueString.append("i");

    return influxValueString;
}

template<>
std::string InfluxClient::constructInfluxValueString(uint value) {
    std::string influxValueString = std::to_string(value);
    influxValueString.append("u");

    return influxValueString;
}

template<>
std::string InfluxClient::constructInfluxValueString(bool value) {
    std::string influxValueString = std::to_string(value);

    return influxValueString;
}

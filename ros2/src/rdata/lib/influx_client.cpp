#include <influx_client.hpp>

namespace influx {
    Client::Client() {
        this->credentials = getCredentials();

        // The POST url defines all the parameters for the current datalogging session
        this->urlWrite = constructInfluxURL();

        // The autorization must be sent as a POST header
        this->authorization = constructInfluxAuthorization();

        // Initialize and configure curl to be able to write to InfluxDB
        curl_global_init(CURL_GLOBAL_ALL);
    }

    Client::~Client() {
        // Must clean up curl on deconstruction of class
        curl_global_cleanup();
    }

    size_t Client::writeCallback(char *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);

        return size * nmemb;
    }

    // Poplate credentials sructure by reading .toml file on the system
    // InfluxDB buckets allow for the user to segregate different data logging sessions within a single database
    // As such the user should be prompted to allow for overiding of the default bucket
    Credentials Client::getCredentials() {
        Credentials credentials;
        toml::table tbl = getToml(CREDENTIALS_FILE);

        credentials.user = getTomlEntryBySectionKey(tbl, "credentials", "user");
        credentials.password = getTomlEntryBySectionKey(tbl, "credentials", "password");
        credentials.token = getTomlEntryBySectionKey(tbl, "credentials", "token");
        credentials.org = getTomlEntryBySectionKey(tbl, "credentials", "org");
        credentials.retention = getTomlEntryBySectionKey(tbl, "default-params", "retention");
        credentials.bucket = getTomlEntryBySectionKey(tbl, "default-params", "bucket");

        return credentials;
    }

    // Print the current credentials
    // Used for debugging
    void Client::printCredentials() {
        std::cout << std::endl;
        std::cout << "User: " << this->credentials.user << std::endl;
        std::cout << "Password: " << this->credentials.password << std::endl;
        std::cout << "Token: " << this->credentials.token << std::endl;
        std::cout << "Org: " << this->credentials.org << std::endl;
        std::cout << "Bucket: " << this->credentials.bucket << std::endl;
        std::cout << "Retention: " << this->credentials.retention << std::endl;

        return;
    }

    long Client::postRequest(std::string &url, struct curl_slist *header, std::string &body, std::string &responeBuffer) {
        CURL *curl = curl_easy_init();
        CURLcode curlResponse;

        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header);
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, Client::writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &responeBuffer);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());

            long responseCode;

            // Perform the POST request and get the response
            curlResponse = curl_easy_perform(curl);

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &responseCode);

            if (curlResponse != CURLE_OK) {
                std::string error = curl_easy_strerror(curlResponse);

                curl_easy_cleanup(curl);

                throw except::Curl(error);
            }

            curl_easy_cleanup(curl);

            return responseCode;
        } else {
            curl_easy_cleanup(curl);

            throw except::Curl("CURL object does not exist.");
        }
    }

    // Use the toml++ librarty to parse a .toml file
    // Returns a toml table class with built in methods to access data
    // REFERENCE: https://marzer.github.io/tomlplusplus/
    toml::table Client::getToml(std::string path) {
        toml::table tbl;

        try {
            tbl = toml::parse_file(path);
        } catch (const toml::parse_error &e) {
            std::stringstream error;

            error << std::endl;
            error << "Cannot parse credentials file!" << std::endl;
            error << "Error: " << e << std::endl;

            throw std::runtime_error(error.str());
        }

        return tbl;
    }

    // Get the entry with in a toml table by section reference and key value
    // toml sections are denoted within the file as '[section]'
    // toml key value pairs are denoted within the file as 'key = "value"'
    // Failure to parse the credentials .toml should result in an unrecoverable error
    std::string Client::getTomlEntryBySectionKey(toml::table tbl, std::string section, std::string key) {
        std::optional<std::string> entry = tbl[section][key].value<std::string>();

        try {
            entry.value();
        } catch (const std::bad_optional_access &e) {
            std::stringstream error;

            error << std::endl;
            error << "No '" << key << "' found in credentials file!" << std::endl;
            error << "File Path: " << CREDENTIALS_FILE << std::endl;
            error << "Section: " << section << std::endl;
            error << "Key: " << key << std::endl;
            error << "Error: " << e.what() << std::endl;
            error << std::endl;

            throw std::runtime_error(error.str());
        }

        return entry.value();
    }

    // Construct the InfluxDB POST url
    std::string Client::constructInfluxURL() {
        std::string url = "http://localhost:8086/api/v2/write?org=";
        url.append(this->credentials.org);
        url.append("&bucket=");
        url.append(this->credentials.bucket);
        url.append("&precision=ns");

        return url;
    }

    // Contsruct the authorization string needed in the header of the POST request for InlfuxDB
    std::string Client::constructInfluxAuthorization() {
        std::string authorization = "Authorization: Token ";
        authorization.append(this->credentials.token);

        return authorization;
    }

    // Construct the value string of the InfluxDB POST request
    // Various overloads are impleneted for the different types of data that InfluxDB can store
    // REFERENCE: https://docs.influxdata.com/influxdb/v2.0/reference/syntax/line-protocol/
    std::string Client::constructInfluxValueString(bool value) {
        std::string influxValueString;
        if (value == true) {
            influxValueString = "TRUE";
        } else {
            influxValueString = "FALSE";
        }

        return influxValueString;
    }

    std::string Client::constructInfluxValueString(double value) {
        std::string influxValueString = std::to_string(value);

        return influxValueString;
    }

    std::string Client::constructInfluxValueString(std::string value) {
        std::string influxValueString = "\"";
        influxValueString.append(value);
        influxValueString.append("\"");

        return influxValueString;
    }

    std::string Client::constructInfluxValueString(int64_t value) {
        std::string influxValueString = std::to_string(value);
        influxValueString.append("i");

        return influxValueString;
    }

    std::string Client::constructInfluxValueString(uint64_t value) {
        std::string influxValueString = std::to_string(value);
        influxValueString.append("u");

        return influxValueString;
    }

    // Construct InfluxDB line protocol for POST request
    // The 'measurment' field indicated the class/category/grouping of the measurment being written
    // The 'sensor' field allows for naming of the specific sensor or origin of the data being written
    // The 'value' feild is the actual data bein logged
    std::string Client::constructInfluxPostBody(std::string measurment, std::string sensor, std::string value) {
        std::string influxPostBody = measurment;

        influxPostBody.append(",sensor=");
        influxPostBody.append(sensor);
        influxPostBody.append(" value=");
        influxPostBody.append(value);

        return influxPostBody;
    }
} // namespace influx

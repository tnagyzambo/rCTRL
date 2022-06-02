#include <rdata/logger.hpp>

namespace rdata {
    Logger::Logger(std::string credentialsFilePath) {
        this->credentials = getCredentials(credentialsFilePath);

        // The POST url defines all the parameters for the current datalogging session
        this->urlWrite = constructInfluxURL();

        // The autorization must be sent as a POST header
        this->authorization = constructInfluxAuthorization();

        // Initialize and configure curl to be able to write to InfluxDB
        curl_global_init(CURL_GLOBAL_ALL);
    }

    Logger::~Logger() {
        // Must clean up curl on deconstruction of class
        curl_global_cleanup();
    }

    size_t Logger::writeCallback(char *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);

        return size * nmemb;
    }

    // Poplate credentials sructure by reading .toml file on the system
    // InfluxDB buckets allow for the user to segregate different data logging sessions within a single database
    // As such the user should be prompted to allow for overiding of the default bucket
    Credentials Logger::getCredentials(std::string credentialsFilePath) {
        Credentials credentials;
        credentials.path = credentialsFilePath;

        toml::table toml = rutil::toml::getToml(credentialsFilePath);

        auto tomlView = toml::node_view(toml);

        toml::node_view<toml::node> credentialsView = rutil::toml::viewOfTable(tomlView, "credentials");
        credentials.user = rutil::toml::getTomlEntryByKey<std::string>(credentialsView, "user");
        credentials.password = rutil::toml::getTomlEntryByKey<std::string>(credentialsView, "password");
        credentials.token = rutil::toml::getTomlEntryByKey<std::string>(credentialsView, "token");
        credentials.org = rutil::toml::getTomlEntryByKey<std::string>(credentialsView, "org");

        toml::node_view<toml::node> paramsView = rutil::toml::viewOfTable(tomlView, "default-params");
        credentials.retention = rutil::toml::getTomlEntryByKey<std::string>(paramsView, "retention");
        credentials.bucket = rutil::toml::getTomlEntryByKey<std::string>(paramsView, "bucket");

        return credentials;
    }

    long Logger::postRequest(std::string &url, struct curl_slist *header, std::string &body, std::string &responeBuffer) {
        CURL *curl = curl_easy_init();
        CURLcode curlResponse;

        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header);
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, Logger::writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &responeBuffer);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());

            long responseCode;

            // Perform the POST request and get the response
            curlResponse = curl_easy_perform(curl);

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &responseCode);

            if (curlResponse != CURLE_OK) {
                std::string error = curl_easy_strerror(curlResponse);

                curl_easy_cleanup(curl);

                throw rdata::except::Curl(error);
            }

            curl_easy_cleanup(curl);

            return responseCode;
        } else {
            curl_easy_cleanup(curl);

            throw except::Curl("CURL object does not exist.");
        }
    }

    // Construct the InfluxDB POST url
    std::string Logger::constructInfluxURL() {
        std::string url = fmt::format("http://localhost:8086/api/v2/write?org={}&bucket={}&precision=ns",
                                      this->credentials.org,
                                      this->credentials.bucket);

        return url;
    }

    // Contsruct the authorization string needed in the header of the POST request for InlfuxDB
    std::string Logger::constructInfluxAuthorization() {
        std::string authorization = "Authorization: Token ";
        authorization.append(this->credentials.token);

        return authorization;
    }

    // Append a line protocol entry to the post body
    void Logger::log(std::string lineProtocol) {
        this->postBody.append(fmt::format("measurement,{} {}\n",
                                          lineProtocol,
                                          duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count()));
    }

    // Implentation of the templated function writeToInflux
    // Write the POST request to InfluxDB
    // The POST request can fail and should be regarded as a non critical error
    // If CURL is incorrectly configured at runtime it should be considered and unrecoverable error
    void Logger::writeToInflux() {
        struct curl_slist *header = NULL;
        header = curl_slist_append(header, this->authorization.c_str());

        std::string responseBuffer;

        long responseCode = this->postRequest(this->urlWrite, header, postBody, responseBuffer);

        if (responseCode != 204) {
            throw except::PostReq(responseBuffer);
        }

        // Reset postBody buffer
        this->postBody = "";
    }
} // namespace rdata

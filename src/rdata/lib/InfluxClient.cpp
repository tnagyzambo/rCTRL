#include <InfluxClient.hpp>

influxclient::Client::Client()
{
    this->credentials = getCredentials();

    // The POST url defines all the parameters for the current datalogging session
    this->url = constructInfluxURL();

    // The autorization must be sent as a POST header
    this->authorization = constructInfluxAuthorization();
    this->headers = curl_slist_append(this->headers, this->authorization.c_str());

    // Initialize and configure curl to be able to write to InfluxDB
    curl_global_init(CURL_GLOBAL_ALL);
    this->curl = curl_easy_init();
    curl_easy_setopt(this->curl, CURLOPT_URL, this->url.c_str());
    curl_easy_setopt(this->curl, CURLOPT_HTTPHEADER, this->headers);
    curl_easy_setopt(this->curl, CURLOPT_POST, 1L);
    //curl_easy_setopt(this->curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(this->curl, CURLOPT_WRITEFUNCTION, influxclient::Client::writeCallback);
    curl_easy_setopt(this->curl, CURLOPT_WRITEDATA, &this->curlReadBuffer);
}

influxclient::Client::~Client()
{
    // Must clean up curl on deconstruction of class
    curl_global_cleanup();
}

size_t influxclient::Client::writeCallback(char *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *)userp)->append((char *)contents, size * nmemb);

    return size * nmemb;
}

// Poplate credentials sructure by reading .toml file on the system
// InfluxDB buckets allow for the user to segregate different data logging sessions within a single database
// As such the user should be prompted to allow for overiding of the default bucket
influxclient::Credentials influxclient::Client::getCredentials()
{
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

// Print the current credentials
// Used for debugging
void influxclient::Client::printCredentials()
{
    std::cout << std::endl;
    std::cout << "\033[1mUser: \033[0m" << this->credentials.user << std::endl;
    std::cout << "\033[1mPassword: \033[0m" << this->credentials.password << std::endl;
    std::cout << "\033[1mToken: \033[0m" << this->credentials.token << std::endl;
    std::cout << "\033[1mOrg: \033[0m" << this->credentials.org << std::endl;
    std::cout << "\033[1mBucket: \033[0m" << this->credentials.bucket << std::endl;
    std::cout << "\033[1mRetention: \033[0m" << this->credentials.retention << std::endl;

    return;
}

// Prompt the user for the name of the bucket that they would like to use
std::string influxclient::Client::promptForBucket(std::string bucket)
{
    std::string input;

    std::cout << std::endl;
    std::cout << "\033[1;36mEnter bucket for current data logging session.\033[0m" << std::endl;
    std::cout << "\033[1mLeave empty to use default bucket \"" << bucket << "\".\033[0m" << std::endl;
    std::cout << "\033[1mBucket: \033[0m";
    std::getline(std::cin, input);

    if (input != "")
    {
        bucket = input;
    }
    else
    {
        std::cout << "\033[A";
        std::cout << "\033[1mBucket: \033[0m";
        std::cout << bucket;
        std::cout << std::endl;
    }

    std::cout << std::endl;

    return bucket;
}

// Use the toml++ librarty to parse a .toml file
// Returns a toml table class with built in methods to access data
// REFERENCE: https://marzer.github.io/tomlplusplus/
toml::table influxclient::Client::getToml(std::string path)
{
    toml::table tbl;

    try
    {
        tbl = toml::parse_file(path);
    }
    catch (const toml::parse_error &e)
    {
        std::stringstream error;

        error << std::endl;
        error << "\033[1;31mCANNOT PARSE CREDENTIALS FILE!\033[0m" << std::endl;
        error << "\033[1mError: \033[0m" << e << std::endl;

        throw std::runtime_error(error.str());
    }

    return tbl;
}

// Get the entry with in a toml table by section reference and key value
// toml sections are denoted within the file as '[section]'
// toml key value pairs are denoted within the file as 'key = "value"'
// Failure to parse the credentials .toml should result in an unrecoverable error
std::string influxclient::Client::getTomlEntryBySectionKey(toml::table tbl, std::string section, std::string key)
{
    std::optional<std::string> entry = tbl[section][key].value<std::string>();

    try
    {
        entry.value();
    }
    catch (const std::bad_optional_access &e)
    {
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

// Construct the InfluxDB POST url
std::string influxclient::Client::constructInfluxURL()
{
    std::string url = "http://localhost:8086/api/v2/write?org=";
    url.append(this->credentials.org);
    url.append("&bucket=");
    url.append(this->credentials.bucket);
    url.append("&precision=ns");

    return url;
}

// Contsruct the authorization string needed in the header of the POST request for InlfuxDB
std::string influxclient::Client::constructInfluxAuthorization()
{
    std::string authorization = "Authorization: Token ";
    authorization.append(this->credentials.token);

    return authorization;
}

// Construct the value string of the InfluxDB POST request
// Various overloads are impleneted for the different types of data that InfluxDB can store
// REFERENCE: https://docs.influxdata.com/influxdb/v2.0/reference/syntax/line-protocol/
std::string influxclient::Client::constructInfluxValueString(bool value)
{
    std::string influxValueString;
    if (value == true)
    {
        influxValueString = "TRUE";
    }
    else
    {
        influxValueString = "FALSE";
    }

    return influxValueString;
}

std::string influxclient::Client::constructInfluxValueString(double value)
{
    std::string influxValueString = std::to_string(value);

    return influxValueString;
}

std::string influxclient::Client::constructInfluxValueString(std::string value)
{
    std::string influxValueString = "\"";
    influxValueString.append(value);
    influxValueString.append("\"");

    return influxValueString;
}

std::string influxclient::Client::constructInfluxValueString(int64_t value)
{
    std::string influxValueString = std::to_string(value);
    influxValueString.append("i");

    return influxValueString;
}

std::string influxclient::Client::constructInfluxValueString(uint64_t value)
{
    std::string influxValueString = std::to_string(value);
    influxValueString.append("u");

    return influxValueString;
}

// Construct InfluxDB line protocol for POST request
// The 'measurment' field indicated the class/category/grouping of the measurment being written
// The 'sensor' field allows for naming of the specific sensor or origin of the data being written
// The 'value' feild is the actual data bein logged
std::string influxclient::Client::constructInfluxPostBody(std::string measurment, std::string sensor, std::string value)
{
    std::string influxPostBody = measurment;

    influxPostBody.append(",sensor=");
    influxPostBody.append(sensor);
    influxPostBody.append(" value=");
    influxPostBody.append(value);

    return influxPostBody;
}
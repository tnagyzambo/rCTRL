// Implentation of the templated function writeToInflux
// Write the POST request to InfluxDB
// The POST request can fail and should be regarded as a non critical error
// If CURL is incorrectly configured at runtime it should be considered and unrecoverable error
template <typename T>
void influxclient::Client::writeToInflux(std::string measurment, std::string sensor, T value)
{
    if (curl)
    {
        long curlResponseCode;

        std::string postBody = constructInfluxPostBody(measurment, sensor, constructInfluxValueString(value));
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postBody.c_str());

        // Perform the POST request and get the response
        this->curlReadBuffer.clear();
        this->curlResponse = curl_easy_perform(curl);

        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &curlResponseCode);

        if (this->curlResponse == CURLE_OK)
        {
            if (curlResponseCode != 204)
            {
                throw influxclient::PostRequestException(this->curlReadBuffer);
            }
        }
        else
        {
            std::string error = curl_easy_strerror(this->curlResponse);
            throw influxclient::CurlException(error);
        }
    }
    else
    {
        throw influxclient::CurlException("CURL object does not exist.");
    }

    return;
}

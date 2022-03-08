// Implentation of the templated function writeToInflux
// Write the POST request to InfluxDB
// The POST request can fail and should be regarded as a non critical error
// If CURL is incorrectly configured at runtime it should be considered and unrecoverable error
template <typename T> void Client::writeToInflux(std::string measurment, std::string sensor, T value) {
    struct curl_slist *header = NULL;
    header = curl_slist_append(header, this->authorization.c_str());

    std::string body = constructInfluxPostBody(measurment, sensor, constructInfluxValueString(value));
    std::string responeBuffer;

    long responseCode = this->postRequest(this->urlWrite, header, body, responeBuffer);

    if (responseCode != 204) {
        throw except::PostReq(responeBuffer);
    }
}

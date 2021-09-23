template <typename T>
void InfluxClient::writeToInflux(std::string measurment, std::string sensor, T value) {
    if (curl) {
        std::string postBody = measurment;
        postBody.append(",sensor=");
        postBody.append(sensor);
        postBody.append(" value=");
        postBody.append(constructInfluxValueString(value));

        std::cout << postBody << std::endl;

        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postBody.c_str());
        
        // Perform the request, res will get the return code
        res = curl_easy_perform(curl);

        if(res != CURLE_OK) {
            std::cout << "\033[1;31m" << stderr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << "\033[0m" << std::endl;
        }

        std::cout << res << std::endl;             
    } else {
        //throw
    }

    return;
}

template <typename T>
std::string InfluxClient::constructInfluxValueString(T value) {
    std::string influxValueString = "\"";
    influxValueString.append(value);
    influxValueString.append("\"");

    return influxValueString;
}

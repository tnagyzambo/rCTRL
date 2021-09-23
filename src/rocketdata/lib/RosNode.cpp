#include "RosNode.hpp"

RosNode::RosNode() : rclcpp::Node("rocketdata") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&RosNode::topic_callback, this, std::placeholders::_1));

    /* get a curl handle */ 
    // this->curl = curl_easy_init();
    // ///this->authorization = "Authorization: Token " + this->influxInfo.token;
    // this->headers = curl_slist_append(this->headers, this->authorization.c_str());
    // curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    // curl_easy_setopt(this->curl, CURLOPT_URL, "http://localhost:8086/api/v2/write?org=ros&bucket=default&precision=ns");
    // curl_easy_setopt(this->curl, CURLOPT_POST, 1L);
    // curl_easy_setopt(this->curl, CURLOPT_VERBOSE, 1L);
    // curl_easy_setopt(this->curl, CURLOPT_WRITEFUNCTION, MinimalSubscriber::WriteCallback);
    // curl_easy_setopt(this->curl, CURLOPT_WRITEDATA, &curlReadBuffer);

    //RCLCPP_INFO(this->get_logger(), "%s", this->influxInfo.user.c_str());
}

size_t RosNode::WriteCallback(char *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);

    return size * nmemb;
}

void RosNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    // if(this->curl) {
    //     std::string postBody = "test,sensor=test value=" + msg->data;

    //     std::cout << postBody << std::endl;

    //     curl_easy_setopt(this->curl, CURLOPT_POSTFIELDS, postBody.c_str());
        

    //     /* Perform the request, res will get the return code */ 
    //     res = curl_easy_perform(curl);
    //     /* Check for errors */ 
    //     if(res != CURLE_OK)
    //     fprintf(stderr, "curl_easy_perform() failed: %s\n",
    //             curl_easy_strerror(res));

    //     std::cout << readBuffer << std::endl;             
    // }
}
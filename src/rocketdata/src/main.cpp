#include <memory>
#include <string>
#include "../lib/InfluxClient.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <stdio.h>
#include <curl/curl.h>

using std::placeholders::_1;
using std::string;



class MinimalSubscriber : public rclcpp::Node {
    public:
        MinimalSubscriber() : Node("minimal_subscriber") {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

            // Example get user from env
            // char* pEnv;
            // pEnv = getenv("ROS_INFLUX_API_TOKEN");
            // if(pEnv == NULL) {
            //     throw std::runtime_error("No 'ROS_INFLUX_API_TOKEN' enviroment variable could be found.");
            // }
            // this->influxInfo.token = pEnv;
            // this->influxInfo.user = "ros";
            // this->influxInfo.password = "ros";
            // this->influxInfo.org = "ros";
            // this->influxInfo.bucket = "default";

            /* get a curl handle */ 
            this->curl = curl_easy_init();
            ///this->authorization = "Authorization: Token " + this->influxInfo.token;
            this->headers = curl_slist_append(this->headers, this->authorization.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(this->curl, CURLOPT_URL, "http://localhost:8086/api/v2/write?org=ros&bucket=default&precision=ns");
            curl_easy_setopt(this->curl, CURLOPT_POST, 1L);
            curl_easy_setopt(this->curl, CURLOPT_VERBOSE, 1L);
            curl_easy_setopt(this->curl, CURLOPT_WRITEFUNCTION, MinimalSubscriber::WriteCallback);
            curl_easy_setopt(this->curl, CURLOPT_WRITEDATA, &curlReadBuffer);

            //RCLCPP_INFO(this->get_logger(), "%s", this->influxInfo.user.c_str());
        }

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

            if(this->curl) {
                string postBody = "test,sensor=test value=" + msg->data;

                std::cout << postBody << std::endl;

                curl_easy_setopt(this->curl, CURLOPT_POSTFIELDS, postBody.c_str());
                

                /* Perform the request, res will get the return code */ 
                res = curl_easy_perform(curl);
                /* Check for errors */ 
                if(res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n",
                        curl_easy_strerror(res));

                std::cout << readBuffer << std::endl;             
            }
        }

        static size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp)
        {
            ((std::string*)userp)->append((char*)contents, size * nmemb);
            return size * nmemb;
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        InfluxClient influxClient;
        string readBuffer;
        CURL *curl;
        CURLcode res;
        std::string curlReadBuffer;
        struct curl_slist* headers = NULL;
        string authorization;
};



int main() {
    //int argc, char *argv[]
    InfluxClient influxClient;
    /* In windows, this will init the winsock stuff */ 
    // curl_global_init(CURL_GLOBAL_ALL);

    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MinimalSubscriber>());
    // rclcpp::shutdown();

    // // curl_slist_free_all(headers);
    // // curl_easy_cleanup(curl);
    // curl_global_cleanup();

    return 0;
}

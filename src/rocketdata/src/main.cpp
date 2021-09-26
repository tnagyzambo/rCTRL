#include <RocketDataNode.hpp>

int main(int argc, char *argv[]) {
    /* In windows, this will init the winsock stuff */ 
    // curl_global_init(CURL_GLOBAL_ALL);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RocketDataNode>());
    rclcpp::shutdown();

    // // curl_slist_free_all(headers);
    // // curl_easy_cleanup(curl);
    // curl_global_cleanup();

    return 0;
}

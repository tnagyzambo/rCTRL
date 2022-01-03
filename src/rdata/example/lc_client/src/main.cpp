#include <lc_client_node.hpp>
#include <fstream>
void test(const rcutils_log_location_t *a, int b, const char *c, long d, const char *e, va_list *f)
{

    (void)b;
    (void)c;
    (void)d;
    (void)e;
    (void)f;

    // std::fstream file;
    // file.open(a->file_name, std::fstream::out);
    // file << "fuuuuck dude \n";
    std::cout << a->file_name << "\n";
    std::cout << a->function_name << "\n";
    std::cout << a->line_number << "\n";
    std::cout << b << "\n";
    std::cout << c << "\n";
    std::cout << d << "\n";
    std::cout << e << "\n";
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //rcutils_logging_set_output_handler(&test);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto lcClient = std::make_shared<rdata::lc_client::Node>("lcClient");

    executor.add_node(lcClient);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

//void (*)(const rcutils_log_location_t *, int, const char *, const char *, __builtin_va_list *)
//void (*)(const rcutils_log_location_t *, int, const char *, long, const char *, __builtin_va_list *)
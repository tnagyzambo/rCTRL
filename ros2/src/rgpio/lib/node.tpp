// Get the entry with in a toml table by key value
// toml key value pairs are denoted within the file as 'key = "value"'
// Failure to parse the config.toml should result in an unrecoverable error
template <typename T>
T rgpio::Node::getTomlEntryByKey(toml::table tbl, std::string key)
{
    std::optional<T> entry = tbl[key].value<T>();

    try
    {
        entry.value();
    }
    catch (const std::bad_optional_access &e)
    {
        // Unrecoverable error
        std::stringstream error;

        error << "\0337\033[1;31mMISSING KEY IN CONFIG FILE!\033[0m";
        error << "\0338\f\0337\033[1;31mFile Path: \033[0m\033[31m" << GPIO_CONFIG_FILE << "\033[0m";
        error << "\0338\f\0337\033[1;31mKey: \033[0m\033[31m" << key << "\033[0m";
        error << "\0338\f\0337\033[1;31mError: \033[0m\033[31m" << e.what() << "\033[0m";

        RCLCPP_FATAL(this->get_logger(), "%s", error.str().c_str());

        throw std::runtime_error(error.str());
    }

    return entry.value();
}
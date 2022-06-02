// TOML++ can return multiple types
// Templated function for toml value access with standardized formating error handling
template <typename T>
T getTomlEntryByKey(::toml::node_view<::toml::node> node_view, const char *key) {
    std::optional<T> entry = node_view[key].value<T>();

    try {
        entry.value();
    } catch (const std::bad_optional_access &e) {
        std::string error =
            ::fmt::format("Unable to parse key in toml section!\nKey: {}\nTOML: {}\nError: {}", key, node_view, e.what());

        throw rutil::except::toml_parse_error(error);
    }

    return entry.value();
}

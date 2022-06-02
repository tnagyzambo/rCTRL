#include <rutil/toml.hpp>

namespace rutil::toml {
    // Use the toml++ librarty to parse a .toml file
    // Returns a toml table class with built in methods to access data
    // REFERENCE: https://marzer.github.io/tomlplusplus/
    ::toml::table getToml(std::string path) {
        ::toml::table tbl;

        try {
            tbl = ::toml::parse_file(path);
        } catch (const ::toml::parse_error &e) {
            std::string error = ::fmt::format("Cannot parse TOML file!\nError: {}", e.what());

            throw rutil::except::toml_parse_error(error);
        }

        return tbl;
    }

    ::toml::node_view<::toml::node> viewOfTable(::toml::node_view<::toml::node> node_view, const char *key) {
        if (node_view[key].as_table() == nullptr) {
            std::string error =
                ::fmt::format("Key in TOML node_view does not return a table!\nKey: {}\nTOML: {}", key, node_view);

            throw rutil::except::toml_parse_error(error);
        }

        return node_view[key];
    }

    ::toml::node_view<::toml::node> viewOfArray(::toml::node_view<::toml::node> node_view, const char *key) {
        if (node_view[key].as_array() == nullptr) {
            std::string error =
                ::fmt::format("Key in TOML node_view does not return a array!\nKey: {}\nTOML: {}", key, node_view);

            throw rutil::except::toml_parse_error(error);
        }

        return node_view[key];
    }
} // namespace rutil::toml

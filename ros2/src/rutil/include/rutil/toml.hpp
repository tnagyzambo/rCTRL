#pragma once

#include <rutil/except.hpp>
#include <toml++/toml.h>

namespace rutil::toml {
    template <typename T>
    ::toml::node_view<T> viewOfTable(::toml::node_view<T> node_view, const char *key) {
        if (node_view[key].as_table() == nullptr) {
            std::stringstream error;

            error << "Key in TOML node_view does not return a table!\n";
            error << "Key: " << key << "\n";
            error << "TOML: " << node_view << "\n";

            throw rutil::except::toml_parse_error(error.str());
        }

        return node_view[key];
    }

    template <typename T>
    ::toml::node_view<T> viewOfArray(::toml::node_view<T> node_view, const char *key) {
        if (node_view[key].as_array() == nullptr) {
            std::stringstream error;

            error << "Key in TOML node_view does not return an array!\n";
            error << "Key: " << key << "\n";
            error << "TOML: " << node_view << "\n";

            throw rutil::except::toml_parse_error(error.str());
        }

        return node_view[key];
    }

    // TOML++ can return multiple types
    // Templated function for toml value access with standardized formating error handling
    template <typename T>
    T getTomlEntryByKey(::toml::node_view<::toml::node> node_view, const char *key) {
        std::optional<T> entry = node_view[key].value<T>();

        try {
            entry.value();
        } catch (const std::bad_optional_access &e) {
            std::stringstream error;

            error << "Unable to parse key in toml section!\n";
            error << "Key: " << key << "\n";
            error << "TOML: " << node_view << "\n";
            error << "Error: " << e.what() << "\n";

            throw rutil::except::toml_parse_error(error.str());
        }

        return entry.value();
    }
} // namespace rutil::toml

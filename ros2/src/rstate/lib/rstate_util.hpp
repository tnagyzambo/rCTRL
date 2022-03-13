#pragma once

#include <rstate_except.hpp>
#include <string>
#include <toml++/toml.h>

namespace rstate::util {
    std::string getServiceName(toml::table);

    toml::array *getCmdsAsArray(toml::table &, const char *);

    // TOML++ can return multiple types
    // Templated function for toml value access with standardized formating error handling
    template <typename T>
    T getTomlEntryByKey(toml::table toml, const char *key) {
        std::optional<T> entry = toml[key].value<T>();

        try {
            entry.value();
        } catch (const std::bad_optional_access &e) {
            std::stringstream error;

            error << "Unable to parse key in toml section!/n";
            error << "Key: " << key << "/n";
            error << "Section: " << toml << "/n";
            error << "Error: " << e.what() << "/n";

            throw rstate::except::config_parse_error(error.str());
        }

        return entry.value();
    }
} // namespace rstate::util

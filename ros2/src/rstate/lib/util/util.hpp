#pragma once

#include <string>
#include <toml++/toml.h>
#include <util/except.hpp>

#include <iostream>

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

            error << "Unable to parse key in toml section!\n";
            error << "Key: " << key << "\n";
            error << "Section: " << toml << "\n";
            error << "Error: " << e.what() << "\n";

            throw rstate::except::config_parse_error(error.str());
        }

        return entry.value();
    }

    // Helper function for getTomlTableBySections()
    void pointTomlToTableBySection(toml::table *, const char *);

    // Template packed function to get a toml table that is nest arbitrarly deep in .toml sections
    template <typename... T>
    toml::table getTomlTableBySections(toml::table toml, T... sections) {
        toml::table table = toml;
        // Fold expression to recursivly get toml tables until all section arguments have been applied
        (pointTomlToTableBySection(&table, sections), ...);
        return table;
    }
} // namespace rstate::util

#pragma once

#include <string>
#include <toml++/toml.h>
#include <util/except.hpp>

#include <iostream>

namespace rstate::util {
    namespace toml {
        template <typename T>
        ::toml::node_view<T> viewOfTable(::toml::node_view<T> node_view, const char *key) {
            if (node_view[key].as_table() == nullptr) {
                std::stringstream error;

                error << "Key in TOML node_view does not return a table!\n";
                error << "Key: " << key << "\n";
                error << "TOML: " << node_view << "\n";

                throw rstate::except::config_parse_error(error.str());
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

                throw rstate::except::config_parse_error(error.str());
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

                throw rstate::except::config_parse_error(error.str());
            }

            return entry.value();
        }

        std::string getServiceName(::toml::node_view<::toml::node>);
    } // namespace toml

    std::string getServiceName(::toml::table &);
    ::toml::array *getCmdsAsArray(::toml::table &, const char *);

    // TOML++ can return multiple types
    // Templated function for toml value access with standardized formating error handling
    template <typename T>
    T getTomlEntryByKey(::toml::table &toml, const char *key) {
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
    void pointTomlToTableBySection(::toml::table *, const char *);

    // Template packed function to get a toml table that is nest arbitrarly deep in .toml sections
    template <typename... T>
    ::toml::table getTomlTableBySections(::toml::table &toml, T... sections) {
        ::toml::table table = toml;
        // Fold expression to recursivly get toml tables until all section arguments have been applied
        (pointTomlToTableBySection(&table, sections), ...);
        return table;
    }
} // namespace rstate::util

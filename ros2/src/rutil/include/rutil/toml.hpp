#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <rutil/except.hpp>
#include <string>
#include <toml++/toml.h>

namespace rutil::toml {
    ::toml::table getToml(std::string);

    ::toml::node_view<::toml::node> viewOfTable(::toml::node_view<::toml::node> node_view, const char *key);

    ::toml::node_view<::toml::node> viewOfArray(::toml::node_view<::toml::node> node_view, const char *key);

    // TOML++ can return multiple types
    // Templated function for toml value access with standardized formating error handling
    template <typename T>
    T getTomlEntryByKey(::toml::node_view<::toml::node> node_view, const char *key);

#include "toml.tpp"
} // namespace rutil::toml

#pragma once

#include <toml++/toml.h>
#include <toml++/toml_node_view.h>
#include <util/util.hpp>

namespace rstate {
    class CmdIface {
    public:
        CmdIface(toml::node_view<toml::node> toml) : toml(toml) {}
        virtual ~CmdIface() {}

        toml::node_view<toml::node> toml;

        virtual void execute() = 0;
        virtual void cancel() = 0;
    };
} // namespace rstate

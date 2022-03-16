#pragma once

#include <string>
#include <toml++/toml.h>
#include <toml++/toml_node_view.h>
#include <util/util.hpp>

namespace rstate {
    class CmdIface {
    public:
        CmdIface(toml::node_view<toml::node> toml, bool allowCancel) : allowCancel(allowCancel) {
            // Must convert to owned string
            std::stringstream tomlText;
            tomlText << toml;
            this->toml = tomlText.str();
        }
        virtual ~CmdIface() {}

        std::string toml;
        bool allowCancel;

        virtual void execute() = 0;
        virtual void cancel() = 0;
    };
} // namespace rstate

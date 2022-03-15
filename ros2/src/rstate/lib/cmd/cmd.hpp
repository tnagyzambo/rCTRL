#pragma once

#include <toml++/toml.h>
#include <util/util.hpp>

namespace rstate {
    class CmdIface {
    public:
        CmdIface(toml::table toml) { this->id = util::getTomlEntryByKey<uint>(toml, "id"); }
        virtual ~CmdIface() {}

        uint id;

        virtual void execute() = 0;
        virtual void cancel() = 0;
    };
} // namespace rstate

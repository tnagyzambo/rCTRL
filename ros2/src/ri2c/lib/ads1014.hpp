#pragma once

#include <chrono>
#include <except.hpp>
#include <fmt/core.h>
#include <rutil/toml.hpp>
#include <sys/ioctl.h>
#include <thread>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

using namespace std::chrono_literals;

namespace ri2c {
    class ADS1014 {
    public:
        ADS1014(::toml::node_view<::toml::node>);
        ~ADS1014();

        int address;

        void init(int);
        float read(int);

    private:
    };
} // namespace ri2c

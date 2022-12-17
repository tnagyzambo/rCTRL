#pragma once

#include <asm-generic/int-ll64.h>
#include <chrono>
#include <except.hpp>
#include <fmt/core.h>
#include <rutil/toml.hpp>
#include <sys/ioctl.h>
#include <thread>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <thermocouple.h>
}

using namespace std::chrono_literals;

namespace ri2c {
#define ADS1014_CONF_REG 0b00000001
#define TCAADDR 0x70

    class ADS1014 {
    public:
        ADS1014(::toml::node_view<::toml::node>);
        virtual ~ADS1014();

        int16_t getRaw(int);
        void init(int);

        __u8 channel;
        __u8 address;
        __u8 conf0;
        __u8 conf1;
        double LSB;

        virtual float read(int) = 0;

    private:
    };

    class PAA_7LC_30BAR : public ADS1014 {
    public:
        PAA_7LC_30BAR(::toml::node_view<::toml::node>);
        ~PAA_7LC_30BAR();

        float read(int) override final;

    private:
    };

    class PA_7LHPC_400BAR : public ADS1014 {
    public:
        PA_7LHPC_400BAR(::toml::node_view<::toml::node>);
        ~PA_7LHPC_400BAR();

        float read(int) override final;

    private:
    };

    class LoadcellBridge : public ADS1014 {
    public:
        LoadcellBridge(::toml::node_view<::toml::node>);
        ~LoadcellBridge();

        float read(int) override final;

    private:
    };

    class M5HB_30BAR : public ADS1014 {
    public:
        M5HB_30BAR(::toml::node_view<::toml::node>);
        ~M5HB_30BAR();

        float read(int) override final;

    private:
    };

    class K_TYPE : public ADS1014 {
    public:
        K_TYPE(::toml::node_view<::toml::node>);
        ~K_TYPE();

        float read(int) override final;

    private:
    };
} // namespace ri2c

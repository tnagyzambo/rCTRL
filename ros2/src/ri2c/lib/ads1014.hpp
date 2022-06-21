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
#include <thermocouple.h>
}

using namespace std::chrono_literals;

namespace ri2c {
    #define ADS1014_CONF_REG 0b00000001

    
    class ADS1014 {
    public:
        ADS1014(::toml::node_view<::toml::node>);
        virtual ~ADS1014();

        void sendConfig(int, int, int, unsigned char*);
        float getRaw(int);

        int address;

        virtual void init(int)  = 0;
        virtual float read(int) = 0;

    private:
    };

    class PAA_7LC_30BAR: public ADS1014 {
    public:
        PAA_7LC_30BAR(::toml::node_view<::toml::node>);
        ~PAA_7LC_30BAR();

        void init(int) override final;
        float read(int) override final;
    private:
    };

    class LoadcellBridge: public ADS1014 {
    public:
        LoadcellBridge(::toml::node_view<::toml::node>);
        ~LoadcellBridge();

        void init(int) override final;
        float read(int) override final;
    private:
    };

    class M5HB_30BAR: public ADS1014 {
    public:
        M5HB_30BAR(::toml::node_view<::toml::node>);
        ~M5HB_30BAR();

        void init(int) override final;
        float read(int) override final;
    private:
    };

    class K_TYPE: public ADS1014 {
    public:
        K_TYPE(::toml::node_view<::toml::node>);
        ~K_TYPE();

        void init(int) override final;
        float read(int) override final;
    private:
    };
} // namespace ri2c

#include <ads1014.hpp>
#include <i2c/smbus.h>
#include <thermocouple.h>

namespace ri2c {
    // High level function definitions
    ADS1014::ADS1014(::toml::node_view<::toml::node> toml) {
        this->channel = std::stoul(rutil::toml::getTomlEntryByKey<std::string>(toml, "channel"), nullptr, 16);
        this->address = std::stoul(rutil::toml::getTomlEntryByKey<std::string>(toml, "address"), nullptr, 16);
    }

    ADS1014::~ADS1014() {}

    void ADS1014::sendConfig(int i2cBus, int reg, int num_bytes, unsigned char *bytes) {
        if (i2c_smbus_write_i2c_block_data(i2cBus, reg, num_bytes, bytes) < 0) {
            std::string error = fmt::format(
                "Failed to configure i2c slave at address '0x{:#x}', channel '0x{:#x}'", this->address, this->channel);

            throw except::i2c_error(error);
        }
    }

    float ADS1014::getRaw(int i2cBus) {
        if (ioctl(i2cBus, I2C_SLAVE, TCAADDR) < 0) {
            std::string error = fmt::format("Failed to set i2c channel on multiplexer '0x{:#x}'", this->address);

            throw except::i2c_error(error);
        }

        i2c_smbus_write_byte(i2cBus, this->channel);

        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format(
                "Failed to set i2c slave at address '0x{:#x}', channel '0x{:#x}'", this->address, this->channel);

            throw except::i2c_error(error);
        }

        __s32 res = i2c_smbus_read_word_data(i2cBus, 0b00000000);
        if (res < 0) {
            std::string error = fmt::format(
                "Failed to read i2c slave at address '0x{:#x}', channel '0x{:#x}'", this->address, this->channel);

            throw except::i2c_error(error);
        }

        // Shift
        uint16_t f;
        std::memcpy(&f, &res, sizeof(f));
        // backwards from what you would expect due to how the adc outputs
        uint16_t MSB = (f & 0xff);
        uint16_t LSB = (f & 0xff00) >> 8;
        // shift bits to the right as we have a 12 bit value in a 16 bit int right justified
        uint16_t out = (MSB << 8 | LSB) >> 4;

        return (float)out;
    }

    void ADS1014::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, TCAADDR) < 0) {
            std::string error = fmt::format("Failed to set i2c channel on multiplexer '0x{:#x}'", this->address);

            throw except::i2c_error(error);
        }

        i2c_smbus_write_byte(i2cBus, this->channel);

        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format(
                "Failed to set i2c slave at address '0x{:#x}', channel '0x{:#x}'", this->address, this->channel);

            throw except::i2c_error(error);
        }

        // Read config from toml and write to config register
        unsigned char configuration[2] = {static_cast<unsigned char>(this->conf0),
                                          static_cast<unsigned char>(this->conf1)};
        this->sendConfig(i2cBus, ADS1014_CONF_REG, 2, configuration);

        // Set LSB (volts) given config
        // gain bits are stored in bit 11,10,9 of conf0
        // xxxx010x =>> >>1 = 0xxxx010
        // 0xxxx010&00000111 =>> 00000 010
        auto PGA_bits = (this->conf0 >> 1) & 0b00000111;
        if (PGA_bits == 0b000) {
            this->LSB = 3e-3;
        } else if (PGA_bits == 0b001) {
            this->LSB = 2e-3;
        } else if (PGA_bits == 0b010) {
            this->LSB = 1e-3;
        } else if (PGA_bits == 0b011) {
            this->LSB = 0.5e-3;
        } else if (PGA_bits == 0b100) {
            this->LSB = 0.25e-3;
        } else {
            this->LSB = 0.125e-3;
        }
    }

    // PAA_7LC_30BAR definitions -----------------------------------
    PAA_7LC_30BAR::PAA_7LC_30BAR(::toml::node_view<::toml::node> toml) : ADS1014(toml) {}
    PAA_7LC_30BAR::~PAA_7LC_30BAR() {}

    float PAA_7LC_30BAR::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        float PS_MAX = 30.0;
        // go to conversion register and get bytes back
        float rawData = this->getRaw(i2cBus);

        // Convert data to useful information
        // Our reference voltage is 6.144V.
        // The LSB is 6.144V/2^12.
        // The LSB is 3mV
        // We also need to multiply by a PS_MAX/4 to as the sensor outputs between .1 and .9 V/V
        long double ps_slope = this->LSB * PS_MAX / 4;
        return ps_slope * rawData - PS_MAX / 8;
    }

    // LoadcellBridge definitions ----------------
    LoadcellBridge::LoadcellBridge(::toml::node_view<::toml::node> toml) : ADS1014(toml) {}
    LoadcellBridge::~LoadcellBridge() {}

    float LoadcellBridge::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        // go to conversion register and get bytes back
        float rawData = this->getRaw(i2cBus);

        // Convert data to useful information
        // Our reference voltage is 0.256 V.
        // The LSB is 6.144V/2^12.
        // The LSB is 3mV
        // For now just outputting volts directly
        // TODO: increase gain and calibrate
        double lc_slope = this->LSB;
        double lc_offset = 0;
        return lc_slope * rawData + lc_offset;
    }

    // M5HB_30BAR definitions ----------------
    M5HB_30BAR::M5HB_30BAR(::toml::node_view<::toml::node> toml) : ADS1014(toml) {}
    M5HB_30BAR::~M5HB_30BAR() {}

    float M5HB_30BAR::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        float PS_MAX = 30;
        // go to conversion register and get bytes back
        float rawData = this->getRaw(i2cBus);

        // Convert data to useful information
        // Our reference voltage is 6.144V.
        // The LSB is 6.144V/2^12.
        // The LSB is 3mV
        // The sensor outputs a value between 0 and 10 volts.
        // We use a voltage divider to bring the voltage within 0-5V
        long double ps_slope = this->LSB * PS_MAX / 5.0; // at 10V
        return ps_slope * rawData;
    }

    // K_TYPE definitions -------------
    K_TYPE::K_TYPE(::toml::node_view<::toml::node> toml) : ADS1014(toml) {}
    K_TYPE::~K_TYPE() {}

    float K_TYPE::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        // go to conversion register and get bytes back
        float rawData = this->getRaw(i2cBus);
        // Convert data to useful information
        // Our reference voltage is 0.256 V.
        // The LSB is 0.256V/2^12.
        // The LSB is 0.125mV

        // I think the library is borked
        // float micro_volts = 0.125*rawData;
        // float temperature = (float)thermocoupleMvToC((unsigned long)micro_volts);
        // Assumed ambient of 15 degrees c

        // Just output V for now
        return this->LSB * rawData;
    }
} // namespace ri2c

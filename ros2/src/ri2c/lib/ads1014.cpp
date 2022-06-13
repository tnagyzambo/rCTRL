#include <ads1014.hpp>
#include <i2c/smbus.h>

namespace ri2c {
    ADS1014::ADS1014(::toml::node_view<::toml::node> toml) {
        this->address = std::stoul(rutil::toml::getTomlEntryByKey<std::string>(toml, "address"), nullptr, 16);
    }

    ADS1014::~ADS1014() {}

    void ADS1014::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        unsigned char test[2] = {0b10000100, 0b10000011};
        if (i2c_smbus_write_i2c_block_data(i2cBus, 0b00000001, 2, test) < 0) {
            std::string error = fmt::format("Failed to configure i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }
    }

    float ADS1014::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        __s32 res = i2c_smbus_read_word_data(i2cBus, 0b00000000);
        if (res < 0) {
            std::string error = fmt::format("Failed to read i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        uint f;
        std::memcpy(&f, &res, sizeof(f));

        return (float)f;
    }
} // namespace ri2c

#include <ads1014.hpp>
#include <i2c/smbus.h>
#include <thermocouple.h>

namespace ri2c {

    // High level function definitions
    ADS1014::ADS1014(::toml::node_view<::toml::node> toml) {
        this->address = std::stoul(rutil::toml::getTomlEntryByKey<std::string>(toml, "address"), nullptr, 16);
    }

    ADS1014::~ADS1014() {}

    void ADS1014::sendConfig(int i2cBus, int reg, int num_bytes, unsigned char* bytes) {
        if (i2c_smbus_write_i2c_block_data(i2cBus, reg, num_bytes, bytes) < 0) {
            std::string error = fmt::format("Failed to configure i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }
    }

    float ADS1014::getRaw(int i2cBus) {
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }
        __s32 res = i2c_smbus_read_word_data(i2cBus, 0b00000000);
        if (res < 0) {
            std::string error = fmt::format("Failed to read i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }
        
        //Shift 
        uint16_t f;
        std::memcpy(&f, &res, sizeof(f));
        // backwards from what you would expect due to how the adc outputs
        uint16_t MSB = (f & 0xff);
        uint16_t LSB = (f & 0xff00)>>8;
        // shift bits to the right as we have a 12 bit value in a 16 bit int right justified
        uint16_t out = (MSB <<8 | LSB)>>4;

        return (float)out;
    }

    // PAA_7LC_30BAR definitions -----------------------------------
    PAA_7LC_30BAR::PAA_7LC_30BAR(::toml::node_view<::toml::node> toml) : ADS1014(toml){}
    PAA_7LC_30BAR::~PAA_7LC_30BAR(){}

    void PAA_7LC_30BAR::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        //Set FSR to 6.144 V, continuous mode, 3.3ksps, rest defaults
        unsigned char configuration[2] = {0b10000000, 0b10000011};
        this->sendConfig(i2cBus, ADS1014_CONF_REG, 2, configuration);
    }

    float PAA_7LC_30BAR::read(int i2cBus) {
        // Do all read functions here includeing conversions to float
        float PS_MAX = 30;
        // go to conversion register and get bytes back
        float rawData = this->getRaw(i2cBus);

        // Convert data to useful information
        // Our reference voltage is 6.144V.
        // The LSB is 6.144V/2^12.
        // The LSB is 3mV
        // We also need to multiply by a PS_MAX/4 to as the sensor outputs between .1 and .9 V/V
        long double ps_slope = (3e-3)*PS_MAX/4;
        return ps_slope*rawData-PS_MAX/8;
    }

    // LoadcellBridge definitions ----------------
    LoadcellBridge::LoadcellBridge(::toml::node_view<::toml::node> toml) : ADS1014(toml){}
    LoadcellBridge::~LoadcellBridge(){}


    void LoadcellBridge::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        //Set FSR to 6.144 V, continuous mode, 1.6ksps, rest defaults
        unsigned char configuration[2] = {0b10000000, 0b10000011};
        this->sendConfig(i2cBus, ADS1014_CONF_REG, 2, configuration);
    }

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
        double lc_slope = 3e-3;
        double lc_offset = 0;
        return lc_slope*rawData + lc_offset;
    }


    // M5HB_30BAR definitions ----------------
    M5HB_30BAR::M5HB_30BAR(::toml::node_view<::toml::node> toml) : ADS1014(toml){}
    M5HB_30BAR::~M5HB_30BAR(){}

    void M5HB_30BAR::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        //Set FSR to 6.144 V, continuous mode, 1.6ksps, rest defaults
        unsigned char configuration[2] = {0b10000000, 0b10000011};
        this->sendConfig(i2cBus, ADS1014_CONF_REG, 2, configuration);
    }

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
        long double ps_slope = (3.0e-3)*PS_MAX/5.0; //at 10V 
        return ps_slope*rawData;

    }

    // K_TYPE definitions -------------
    K_TYPE::K_TYPE(::toml::node_view<::toml::node> toml) : ADS1014(toml){}
    K_TYPE::~K_TYPE(){}

    void K_TYPE::init(int i2cBus) {
        // Do all init functions here
        // If they are not the same for each sensor, make this function virtual and implement in unique superclasses that inherit ADS1014
        if (ioctl(i2cBus, I2C_SLAVE, this->address) < 0) {
            std::string error = fmt::format("Failed to set i2c slave at address '0x{:x}'", this->address);

            throw except::i2c_error(error);
        }

        // Set FSR to 0.256 V, continuous mode, 1.6ksps, rest defaults
        // Outputs between -6.4 to 55 mV
        unsigned char configuration[2] = {0b10001010, 0b10000011};
        this->sendConfig(i2cBus, ADS1014_CONF_REG, 2, configuration);
    }

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


        //Just output V for now
        return (0.125e-3)*rawData;
    }
} // namespace ri2c


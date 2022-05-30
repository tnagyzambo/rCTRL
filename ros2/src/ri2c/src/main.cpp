extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main() {
    int file = open("/dev/i2c-1", O_RDWR);

    if (file < 0) {
        std::cout << "failed to open file\n";
        return 1;
    }

    int addr = 0x04;

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cout << "failed to set i2c slave\n";
        return 1;
    }

    i2c_smbus_write_byte(file, 1);
    sleep(1);

    __u8 reg = 0x00; /* Device register to access */
    __u8 res;

    /* Using SMBus commands */

    res = i2c_smbus_read_byte_data(file, reg);
    if (res < 0) {
        std::cout << "failed to read\n";
        return 1;
    }

    uint f;
    std::memcpy(&f, &res, sizeof(f));

    std::cout << "read: " << f;

    close(file);
    return 0;
}

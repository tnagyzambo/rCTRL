#include <helper.hpp>

rgpio::gpio::IOException::IOException() : std::runtime_error("rocketGPIO has failed an IO operation!") {}

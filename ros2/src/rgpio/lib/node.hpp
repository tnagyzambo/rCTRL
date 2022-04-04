#pragma once

#include <gpio.hpp>
#include <gpiod.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <toml++/toml.h>
#include <vector>

#define GPIO_CONFIG_FILE "/home/ros/rocketGPIO/config.toml"

namespace rgpio {
    class Node : public rclcpp::Node {
    public:
        Node();
        ~Node();

    private:
        std::vector<std::unique_ptr<rgpio::gpio::Iface>> gpio;

        toml::table getToml(std::string);
        std::vector<std::unique_ptr<rgpio::gpio::Iface>> getGpio(toml::table);

        template <typename T>
        T getTomlEntryByKey(toml::table, std::string);

        std::unique_ptr<rgpio::gpio::Iface> constructGpio(toml::table);
        std::unique_ptr<rgpio::gpio::Iface> constructInput(std::string,
                                                           rgpio::gpio::chip_number,
                                                           rgpio::gpio::line_number,
                                                           bool);
        std::unique_ptr<rgpio::gpio::Iface> constructInputInterupt(std::string,
                                                                   rgpio::gpio::chip_number,
                                                                   rgpio::gpio::line_number,
                                                                   bool);
        std::unique_ptr<rgpio::gpio::Iface> constructOutput(std::string,
                                                            rgpio::gpio::chip_number,
                                                            rgpio::gpio::line_number,
                                                            bool);
    };
#include <node.tpp>
} // namespace rgpio

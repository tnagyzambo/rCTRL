#pragma once

#include <gpiod.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rgpio/except.hpp>
#include <rgpio/gpio/util.hpp>

// This class implements all functions needed for accessing physical IOs, all real IOs will inherit this class
namespace rgpio {
    namespace gpio {
        class Real : public Gpio {
        public:
            Real(rclcpp::Node *, std::string, chip_number, line_number, line_level::level);
            ~Real();

            void setLineAsInput();
            void setLineAsOutput();

            line_level::level readLine();
            void setLine(line_level::level);

        private:
            rclcpp::Node *node;
            std::string name;

            chip_number chipNumber;
            line_number lineNumber;

            struct gpiod_chip *chip;
            struct gpiod_line *line;

            line_level::level defaultLevel;

            struct gpiod_chip *getChip(chip_number);
            struct gpiod_line *getLine(line_number);

            void releaseLine();
        };
    } // namespace gpio
} // namespace rgpio

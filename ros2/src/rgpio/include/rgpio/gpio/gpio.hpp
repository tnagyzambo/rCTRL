#pragma once

namespace rgpio {
    namespace gpio {
        // Wrap these basic parameters in a struct so we can get compile time
        // checks that we are not passing chipNumber into lineNumber or vice-vera
        struct chip_number {
            explicit chip_number(unsigned int v) : value(v) {}
            unsigned int value;
        };

        struct line_number {
            explicit line_number(unsigned int v) : value(v) {}
            unsigned int value;
        };

        // Line level enum with various conversions
        // The goal is to prevent undefined behaviour caused by casting out of range ints
        // THIS DOESNT WORK - UNDEFINED RUNTIME BEHAVIOUR WHEN OUT OF RANGE
        struct line_level {
            enum level { LOW, HIGH };

            static const char *toStr(const line_level::level &level) {
                switch (level) {
                case LOW:
                    return "LOW";
                case HIGH:
                    return "HIGH";
                default:
                    __builtin_unreachable();
                }
                return "";
            }

            static int toInt(const line_level::level &level) {
                switch (level) {
                case LOW:
                    return 0;
                case HIGH:
                    return 1;
                default:
                    __builtin_unreachable();
                }
                return -1;
            }
        };

        // All IOs must inherit from this interface to be able to be collected in a single vector
        // All methods available for IOs should be defined here
        class Gpio {
        public:
            Gpio() {}
            virtual ~Gpio() {}

            virtual void setLineAsInput() = 0;
            virtual void setLineAsOutput() = 0;

            virtual gpio::line_level::level readLine() = 0;
            virtual void setLine(gpio::line_level::level) = 0;

        private:
        };
    } // namespace gpio
} // namespace rgpio

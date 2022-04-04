#pragma once

#include <helper.hpp>

// All IOs must inherit from this interface to be able to be collected in a single vector
// All methods available for IOs should be defined here
namespace rgpio {
    namespace gpio {
        class Iface {
        public:
            Iface() {}
            virtual ~Iface() {}

            virtual void setLineAsInput() = 0;
            virtual void setLineAsOutput() = 0;

            virtual line_level::level readLine() = 0;
            virtual void setLine(line_level::level) = 0;

        private:
        };
    } // namespace gpio
} // namespace rgpio

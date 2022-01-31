#pragma once

#include <array>
#include <string_view>

// Helper library to format ROS2 logging
namespace rctrl::util::fmt
{
    // Definitions of ANSI escape sequences
    static constexpr std::string_view asciiResetFmt = "\033[0m";
    static constexpr std::string_view asciiRed = "\033[0;31m";
    static constexpr std::string_view asciiRedBold = "\033[1;31m";
    static constexpr std::string_view asciiGreen = "\033[0;32m";
    static constexpr std::string_view asciiGreenBold = "\033[1;32m";
    static constexpr std::string_view asciiMagenta = "\033[0;35m";
    static constexpr std::string_view asciiMagentaBold = "\033[1;35m";
    static constexpr std::string_view asciiCyan = "\033[0;36m";
    static constexpr std::string_view asciiCyanBold = "\033[1;36m";
    static constexpr std::string_view asciiCursorSavePos = "\0337";
    static constexpr std::string_view asciiCursorLoadPos = "\0338";
    static constexpr std::string_view asciiPageFeed = "\f";

    // Internal namespace to hold implementations
    namespace internal
    {
        // Variadic template accepts an arbitrary number of string_view arguments
        // REFERENCE: https://stackoverflow.com/questions/38955940/how-to-concatenate-static-strings-at-compile-time
        template <std::string_view const &...STRS>
        struct joinImpl
        {
            // Join all strings into a single std::array of chars
            static constexpr auto impl() noexcept
            {
                // Fold expression to compute the length of the joined chars
                // Consider the null termination character required by cstrings
                constexpr std::size_t len = (STRS.size() + ... + 0);

                // The 'len + 1' is required by the compiler due to:
                // 'assignment to dereferenced one-past-the-end pointer is not allowed in a constant expression'
                std::array<char, len + 1> arr{};

                // Lambda expression to create function append()
                auto append = [i = 0, &arr](auto const &s) mutable
                {
                    for (auto c : s)
                    {
                        arr[i++] = c;
                    }
                };

                // Fold expression to call append() on all string_view arguements
                (append(STRS), ...);

                // Add null termination character
                arr[len] = 0;

                return arr;
            }

            // Give the joined string static storage
            static constexpr auto arr = impl();

            // View as a std::string_view
            // Remove the last entry from the view that was added for padding
            static constexpr std::string_view value{arr.data(), arr.size() - 1};
        };
    }

    // Wrapper to pull out joined string_view from joinImpl
    template <std::string_view const &...STRS>
    static constexpr auto join = internal::joinImpl<STRS...>::value;

    template <std::string_view const &STR>
    constexpr auto tealBold{join<asciiCyanBold, STR, asciiResetFmt>};

    namespace internal
    {
        // Save the cursor pos at the start of the string
        template <std::string_view const &STR>
        constexpr auto headerLine{join<asciiCursorSavePos, STR>};

        // Return the cursor pos to the saved pos and drop down one line, save the new pos
        template <std::string_view const &STR>
        constexpr auto bodyLine{join<asciiCursorLoadPos, asciiPageFeed, asciiCursorSavePos, STR>};
    }

    // Multiline output, maintain output indentation
    // Accepts one header string and an arbitrary number of body strings
    template <std::string_view const &STR, std::string_view const &...STRS>
    constexpr auto multiLine{join<internal::headerLine<STR>, internal::bodyLine<STRS>...>};
}

// Formatting for ROS2 lifecycle states
namespace rctrl::util::fmt::state
{
    // Internal namespace to hold implementations
    namespace internal
    {
        static constexpr std::string_view strPrepend = "Node is ";
        static constexpr std::string_view strUnconfigured = "Unconfigured";
        static constexpr std::string_view strInactive = "Inactive";
        static constexpr std::string_view strActive = "Active";
        static constexpr std::string_view strFinalized = "Finalized";

        template <std::string_view const &STR>
        constexpr auto fmtState{join<asciiCyan, strPrepend, asciiCyanBold, STR, asciiResetFmt>};
    }

    constexpr auto unconfigured{internal::fmtState<internal::strUnconfigured>.data()};
    constexpr auto inactive{internal::fmtState<internal::strInactive>.data()};
    constexpr auto active{internal::fmtState<internal::strActive>.data()};
    constexpr auto finalized{internal::fmtState<internal::strFinalized>.data()};
}

// Formatting for ROS2 lifecycle transitions
namespace rctrl::util::fmt::transition
{
    // Internal namespace to hold implementations
    namespace internal
    {
        static constexpr std::string_view strPrepend = "Node is ";
        static constexpr std::string_view strConstructing = "Constructing";
        static constexpr std::string_view strConfiguring = "Configuring";
        static constexpr std::string_view strCleaningUp = "Cleaning Up";
        static constexpr std::string_view strShuttingDown = "Shutting Down";
        static constexpr std::string_view strActivating = "Activating";
        static constexpr std::string_view strDeactivating = "Deactivating";
        static constexpr std::string_view strErrorProccessing = "Error Processing";
        static constexpr std::string_view strDestructing = "Destructing";

        template <std::string_view const &STR>
        constexpr auto fmtTransition{join<asciiMagenta, strPrepend, asciiMagentaBold, STR, asciiResetFmt>};
    }

    constexpr auto constructing{internal::fmtTransition<internal::strConstructing>.data()};
    constexpr auto configuring{internal::fmtTransition<internal::strConfiguring>.data()};
    constexpr auto cleaningUp{internal::fmtTransition<internal::strCleaningUp>.data()};
    constexpr auto shuttingDown{internal::fmtTransition<internal::strShuttingDown>.data()};
    constexpr auto activating{internal::fmtTransition<internal::strActivating>.data()};
    constexpr auto deactivating{internal::fmtTransition<internal::strDeactivating>.data()};
    constexpr auto errorProcessing{internal::fmtTransition<internal::strErrorProccessing>.data()};
    constexpr auto destructing{internal::fmtTransition<internal::strDestructing>.data()};
}

namespace rctrl::util::fmt::srv
{
    namespace internal
    {
        static constexpr std::string_view strCreated = "Created service: ";

        template <std::string_view const &STR>
        constexpr auto fmtCreated{join<strCreated, asciiGreen, STR, asciiResetFmt>};
    }

    template <std::string_view const &STR>
    constexpr auto created{internal::fmtCreated<STR>.data()};
}

// Formating for ROS2 subscription operations
namespace rctrl::util::fmt::sub
{
    // Internal namespace to hold implementations
    namespace internal
    {
        static constexpr std::string_view strCreated = "Created subscription: ";
        static constexpr std::string_view strRemoved = "Removed subscription: ";

        template <std::string_view const &STR>
        constexpr auto fmtCreated{join<strCreated, asciiGreen, STR, asciiResetFmt>};

        template <std::string_view const &STR>
        constexpr auto fmtRemoved{join<strCreated, asciiRed, STR, asciiResetFmt>};
    }

    // Run time formatting
    namespace rt
    {
        inline std::string created(const char *topic)
        {
            std::string output = internal::strCreated.data();
            output.append(asciiGreen.data());
            output.append(topic);
            output.append(asciiResetFmt.data());

            return output;
        }

        inline std::string removed(const char *topic)
        {
            std::string output = internal::strRemoved.data();
            output.append(asciiRed.data());
            output.append(topic);
            output.append(asciiResetFmt.data());

            return output;
        }
    }

    template <std::string_view const &STR>
    constexpr auto created{internal::fmtCreated<STR>.data()};

    template <std::string_view const &STR>
    constexpr auto removed{internal::fmtRemoved<STR>.data()};
}

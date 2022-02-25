#pragma once

#include <array>
#include <string_view>

// Helper library to format ROS2 logging
namespace rctrl::util::fmt
{
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
        constexpr auto fmtState{join<strPrepend, STR>};
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
        constexpr auto fmtTransition{join<strPrepend, STR>};
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
        constexpr auto fmtCreated{join<strCreated, STR>};
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
        constexpr auto fmtCreated{join<strCreated, STR>};

        template <std::string_view const &STR>
        constexpr auto fmtRemoved{join<strCreated, STR>};
    }

    // Run time formatting
    namespace rt
    {
        inline std::string created(const char *topic)
        {
            std::string output = internal::strCreated.data();
            output.append(topic);

            return output;
        }

        inline std::string removed(const char *topic)
        {
            std::string output = internal::strRemoved.data();
            output.append(topic);

            return output;
        }
    }

    template <std::string_view const &STR>
    constexpr auto created{internal::fmtCreated<STR>.data()};

    template <std::string_view const &STR>
    constexpr auto removed{internal::fmtRemoved<STR>.data()};
}

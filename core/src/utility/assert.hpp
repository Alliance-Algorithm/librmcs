#pragma once

#include <source_location>

#ifdef NDEBUG
# include <utility>
#endif

namespace librmcs::core::utility {

[[noreturn]] void assert_func(const std::source_location& location);

constexpr inline void
    assert_failed_always(const std::source_location& location = std::source_location::current()) {
    assert_func(location);
}

[[noreturn]] inline void
    assert_failed(const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    (void)location;
    std::unreachable();
#else
    assert_func(location);
#endif
}

constexpr inline void assert_always(
    bool condition, const std::source_location& location = std::source_location::current()) {
    if (!condition) [[unlikely]]
        assert_func(location);
}

constexpr inline void assert_debug(
    bool condition, const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    [[assume(condition)]];
    (void)location;
#else
    assert_always(condition, location);
#endif
}

} // namespace librmcs::core::utility

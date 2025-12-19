#pragma once

#include <source_location>

#ifdef NDEBUG
# include <utility>
#else
# include <exception>
# include <iostream>
# include <print>
#endif

#ifdef assert
# undef assert
#endif

namespace librmcs::core::utility {

[[noreturn]] inline void
    assert_failed(const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    (void)location;
    std::unreachable();
#else
    std::println(
        std::cerr, "Assertion failed at {}:{} in function {}", location.file_name(),
        location.line(), location.function_name());
    std::terminate();
#endif
}

constexpr inline void assert_always(
    bool condition, const std::source_location& location = std::source_location::current()) {
    if (!condition) [[unlikely]]
        assert_failed(location);
}

constexpr inline void
    assert(bool condition, const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    [[assume(condition)]];
    (void)condition;
    (void)location;
#else
    assert_always(condition, location);
#endif
}

} // namespace librmcs::core::utility

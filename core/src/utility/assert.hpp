#pragma once

#include <source_location>
#include <type_traits>
#include <utility>

#ifndef LIBRMCS_DEBUG_ASSERTS
# error "LIBRMCS_DEBUG_ASSERTS must be defined"
#endif

#if LIBRMCS_DEBUG_ASSERTS
# include <functional>
#endif

namespace librmcs::core::utility {

[[noreturn]] void assert_func(const std::source_location& location);

[[noreturn]] constexpr void
    assert_failed_always(const std::source_location& location = std::source_location::current()) {
    assert_func(location);
}

[[noreturn]] inline void
    assert_failed_debug(const std::source_location& location = std::source_location::current()) {
#if LIBRMCS_DEBUG_ASSERTS
    assert_func(location);
#else
    (void)location;
    std::unreachable();
#endif
}

constexpr void assert_always(
    bool condition, const std::source_location& location = std::source_location::current()) {
    if (!condition) [[unlikely]]
        assert_func(location);
}

constexpr void assert_debug(
    bool condition, const std::source_location& location = std::source_location::current()) {
#if LIBRMCS_DEBUG_ASSERTS
    assert_always(condition, location);
#else
    [[assume(condition)]];
    (void)location;
#endif
}

// Debug-only lazy assertion: The predicate is evaluated only in debug builds.
template <typename Condition>
requires std::is_nothrow_invocable_r_v<bool, Condition&&> inline void assert_debug_lazy(
    Condition&& condition, const std::source_location& location = std::source_location::current()) {
#if LIBRMCS_DEBUG_ASSERTS
    assert_always(static_cast<bool>(std::invoke(std::forward<Condition>(condition))), location);
#else
    (void)std::forward<Condition>(condition);
    (void)location;
#endif
}

} // namespace librmcs::core::utility

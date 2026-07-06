#pragma once

#ifndef LIBRMCS_DEBUG_ASSERTS
# error "LIBRMCS_DEBUG_ASSERTS must be defined"
#endif

namespace librmcs::firmware::utility {

[[noreturn, gnu::always_inline]] inline void assert_failed_always() { __builtin_trap(); }

[[gnu::always_inline]] inline void assert_always(bool condition) {
    if (!condition) [[unlikely]]
        assert_failed_always();
}

[[gnu::always_inline]] inline void assert_debug(bool condition) {
#if LIBRMCS_DEBUG_ASSERTS
    assert_always(condition);
#else
    [[assume(condition)]];
#endif
}

} // namespace librmcs::firmware::utility

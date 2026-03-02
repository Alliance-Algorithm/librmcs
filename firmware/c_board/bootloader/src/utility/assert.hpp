#pragma once

namespace librmcs::firmware::utility {

[[noreturn, gnu::always_inline]] inline void assert_failed_always() { __builtin_trap(); }

[[gnu::always_inline]] inline void assert_always(bool condition) {
    if (!condition) [[unlikely]]
        assert_failed_always();
}

[[gnu::always_inline]] inline void assert_debug(bool condition) {
#ifdef NDEBUG
    [[assume(condition)]];
#else
    assert_always(condition);
#endif
}

} // namespace librmcs::firmware::utility

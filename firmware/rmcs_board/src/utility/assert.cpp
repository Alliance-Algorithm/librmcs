#include "core/src/utility/assert.hpp"

#include <source_location>

namespace librmcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

[[noreturn]] void assert_func(const std::source_location& location) {
    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    __builtin_trap();
}

} // namespace librmcs::core::utility

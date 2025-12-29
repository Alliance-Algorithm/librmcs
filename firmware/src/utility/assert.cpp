#include "core/src/utility/assert.hpp"

#include <source_location>

namespace librmcs::core::utility {

volatile const char* assert_file = nullptr;
volatile unsigned int assert_line = 0;
volatile const char* assert_function = nullptr;

[[noreturn]] void assert_func(const std::source_location& location) {
    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    __builtin_trap();
}

} // namespace librmcs::core::utility

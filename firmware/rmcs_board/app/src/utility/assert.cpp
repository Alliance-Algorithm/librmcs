#include "core/src/utility/assert.hpp"

#include <source_location>

#include <hpm_csr_regs.h>
#include <riscv/riscv_core.h>

#include "firmware/rmcs_board/app/src/led/status_led.hpp"

namespace librmcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

[[noreturn]] void assert_func(const std::source_location& location) {
    clear_csr(CSR_MSTATUS, CSR_MSTATUS_MIE_MASK);

    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    firmware::led::StatusLed::show_assertion();
    __builtin_trap();
}

} // namespace librmcs::core::utility

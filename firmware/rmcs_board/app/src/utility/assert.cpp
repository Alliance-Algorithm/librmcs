#include "core/src/utility/assert.hpp"

#include <source_location>

#include <board.h>
#include <hpm_csr_regs.h>
#include <hpm_soc.h>

#include "firmware/rmcs_board/app/src/led/ws2812.hpp"

namespace librmcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

[[noreturn]] void assert_func(const std::source_location& location) {
    disable_global_irq(CSR_MSTATUS_MIE_MASK);

    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    if (auto* ws2812 = firmware::led::ws2812.try_get()) {
        for (int i = 0; i < 10; i++)
            board_delay_ms(10);
        ws2812->set_value(255, 0, 0);
    }

    __builtin_trap();
}

} // namespace librmcs::core::utility

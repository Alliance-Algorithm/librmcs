#pragma once

#include <cstdarg>

#include <SEGGER_RTT.h>

#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::logger {

class Logger {
public:
    using Lazy = utility::Lazy<Logger>;

    Logger() { SEGGER_RTT_Init(); }

    // Non-static to ensure instantiation
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static, cert-dcl50-cpp)
    int printf(const char* fmt, ...) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        va_list args;
        va_start(args, fmt);
        const int n = SEGGER_RTT_vprintf(0, fmt, &args);
        va_end(args);
        return n;
    }

private:
};

inline constinit Logger::Lazy logger;

} // namespace librmcs::firmware::logger

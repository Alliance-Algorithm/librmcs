#pragma once

#include <cstdio>
#include <format>
#include <iostream>
#include <print>
#include <string_view>
#include <utility>

#include "core/src/utility/assert.hpp"

namespace librmcs::host::logging {

enum class Level : int {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERR = 4,
    CRITICAL = 5,
    OFF = 6,
};

#ifndef LIBRMCS_LOGGING_LEVEL
# define LIBRMCS_LOGGING_LEVEL INFO
#endif

class Logger {
public:
    static constexpr Level logging_level = Level::LIBRMCS_LOGGING_LEVEL;

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

public: // Singleton
    static Logger& get_instance() noexcept {
        static Logger logger{};
        return logger;
    }

public: // Logging
    static constexpr bool should_log(Level level) { return level >= logging_level; }

public: // Logging.Formatted
    template <typename... Args>
    void trace(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::TRACE, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::DEBUG, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::INFO, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::WARN, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::ERR, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::CRITICAL, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(Level level, std::format_string<Args...> fmt, Args&&... args) {
        log_internal(level, fmt, std::forward<Args>(args)...);
    }

public: // Logging.Raw
    template <typename T>
    void trace(const T& msg) {
        log_internal(Level::TRACE, msg);
    }

    template <typename T>
    void debug(const T& msg) {
        log_internal(Level::DEBUG, msg);
    }

    template <typename T>
    void info(const T& msg) {
        log_internal(Level::INFO, msg);
    }

    template <typename T>
    void warn(const T& msg) {
        log_internal(Level::WARN, msg);
    }

    template <typename T>
    void error(const T& msg) {
        log_internal(Level::ERR, msg);
    }

    template <typename T>
    void critical(const T& msg) {
        log_internal(Level::CRITICAL, msg);
    }

    template <typename T>
    void log(Level level, const T& msg) {
        log_internal(level, msg);
    }

private:
    constexpr Logger() noexcept = default;

    template <typename... Args>
    void log_internal(Level level, std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(level))
            return;

        print_prefix(level);
        std::println(std::cerr, fmt, std::forward<Args>(args)...);
    }

    template <typename T>
    void log_internal(Level level, const T& msg) {
        if (!should_log(level))
            return;

        print_prefix(level);
        std::cerr << msg << '\n';
    }

    static void print_prefix(Level level) {
        std::string_view level_text = [level]() constexpr -> std::string_view {
            if (level == Level::TRACE)
                return "trace";
            else if (level == Level::DEBUG)
                return "debug";
            else if (level == Level::INFO)
                return "info";
            else if (level == Level::WARN)
                return "warn";
            else if (level == Level::ERR)
                return "error";
            else if (level == Level::CRITICAL)
                return "critical";
            else
                core::utility::assert_failed_debug();
        }();
        std::print(std::cerr, "[librmcs] [{}] ", level_text);
    }
};

inline Logger& get_logger() { return Logger::get_instance(); }

} // namespace librmcs::host::logging

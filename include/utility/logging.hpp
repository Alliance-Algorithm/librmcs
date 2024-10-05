#pragma once

#include <cstdio>

#define LOG_INFO(format, ...) std::fprintf(stdout, "[INFO] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#define LOG_WARN(format, ...) std::fprintf(stderr, "[WARN] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#define LOG_ERROR(format, ...) std::fprintf(stderr, "[ERROR] " format "\n" __VA_OPT__(, ) __VA_ARGS__)
#pragma once

#define LIBRMCS_VERIFY(_cond, _ret) \
    do {                            \
        if (!(_cond)) {             \
            return _ret;            \
        }                           \
    } while (false)

#define LIBRMCS_VERIFY_LIKELY(_cond, _ret) \
    do {                                     \
        if (!(_cond)) [[unlikely]] {         \
            return _ret;                     \
        }                                    \
    } while (false)

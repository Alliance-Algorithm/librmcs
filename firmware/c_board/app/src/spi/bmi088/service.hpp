#pragma once

#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"

namespace librmcs::firmware::spi::bmi088 {

inline void service_pending_reads() {
    if (gyroscope->service_pending_read())
        return;
    accelerometer->service_pending_read();
}

} // namespace librmcs::firmware::spi::bmi088

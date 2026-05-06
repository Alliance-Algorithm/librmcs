#pragma once

#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/temperature.hpp"

namespace librmcs::firmware::spi::bmi088 {

inline void service_pending_reads() {
    if (gyroscope->service_pending_read())
        return;
    if (accelerometer->service_pending_read())
        return;
    temperature->service_pending_read();
}

} // namespace librmcs::firmware::spi::bmi088

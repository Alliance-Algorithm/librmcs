#include "firmware/c_board/app/src/i2c/i2c.hpp"

#include <i2c.h>

#include "core/src/utility/assert.hpp"

namespace librmcs::firmware::i2c {

namespace {

I2c& get_i2c_instance(I2C_HandleTypeDef* hal_i2c_handle) {
    if (hal_i2c_handle == &hi2c2)
        return *i2c0;

    core::utility::assert_failed_debug();
}

} // namespace

extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    get_i2c_instance(hi2c).tx_complete_callback();
}

extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    get_i2c_instance(hi2c).rx_complete_callback();
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    get_i2c_instance(hi2c).tx_complete_callback();
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    get_i2c_instance(hi2c).rx_complete_callback();
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    get_i2c_instance(hi2c).error_callback();
}

} // namespace librmcs::firmware::i2c

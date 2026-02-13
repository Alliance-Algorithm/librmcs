#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#include <board.h>
#include <hpm_gpio_regs.h>
#include <hpm_soc.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/src/spi/spi.hpp"
#include "firmware/rmcs_board/src/usb/vendor.hpp"
#include "firmware/rmcs_board/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

class Accelerometer final
    : private ISpiModule
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Accelerometer, Spi::Lazy*, ChipSelectPin>;

    enum class Range : uint8_t { _3G = 0x00, _6G = 0x01, _12G = 0x02, _24G = 0x03 };
    enum class DataRate : uint8_t {
        _12 = 0x05,
        _25 = 0x06,
        _50 = 0x07,
        _100 = 0x08,
        _200 = 0x09,
        _400 = 0x0A,
        _800 = 0x0B,
        _1600 = 0x0C,
    };

    explicit Accelerometer(
        Spi::Lazy* spi, ChipSelectPin chip_select, Range range = Range::_6G,
        DataRate data_rate = DataRate::_1600)
        : ISpiModule(chip_select)
        , spi_(spi->init()) {

        core::utility::assert_debug(spi_.try_lock());

        auto read_blocked = [this](RegisterAddress address) {
            spi_.transmit_receive_blocked(*this, prepare_tx_buffer_read(address, 1));
            return static_cast<uint8_t>(spi_.rx_buffer[2]);
        };
        auto write_blocked = [this](RegisterAddress address, uint8_t value) {
            spi_.transmit_receive_blocked(*this, prepare_tx_buffer_write(address, value));
        };

        constexpr int max_try_time = 3;
        auto read_with_confirm = [&](RegisterAddress address, uint8_t value) {
            for (int i = max_try_time; i-- > 0;) {
                if (read_blocked(address) == value)
                    return true;
                board_delay_ms(1);
            }
            return false;
        };
        auto write_with_confirm = [&](RegisterAddress address, uint8_t value) {
            for (int i = max_try_time; i-- > 0;) {
                write_blocked(address, value);
                board_delay_ms(1);
                if (read_blocked(address) == value)
                    return true;
            }
            return false;
        };

        // Dummy read to switch accelerometer to SPI mode.
        read_blocked(RegisterAddress::ACC_CHIP_ID);
        board_delay_ms(1);

        // Reset all registers to reset value.
        write_blocked(RegisterAddress::ACC_SOFTRESET, 0xB6);
        board_delay_ms(1);

        // "Who am I" check.
        core::utility::assert_always(read_with_confirm(RegisterAddress::ACC_CHIP_ID, 0x1E));

        // Enable INT1 as output pin, push-pull, active-low.
        core::utility::assert_always(write_with_confirm(RegisterAddress::INT1_IO_CTRL, 0b00001000));
        // Map data ready interrupt to pin INT1.
        core::utility::assert_always(write_with_confirm(RegisterAddress::INT_MAP_DATA, 0b00000100));

        // Set ODR (output data rate) = data_rate and OSR (over-sampling-ratio) = 1.
        core::utility::assert_always(write_with_confirm(
            RegisterAddress::ACC_CONF,
            0x80 | (0x02 << 4) | (static_cast<uint8_t>(data_rate) << 0)));
        // Set accelerometer range.
        core::utility::assert_always(
            write_with_confirm(RegisterAddress::ACC_RANGE, static_cast<uint8_t>(range)));

        // Switch the accelerometer into active mode.
        core::utility::assert_always(write_with_confirm(RegisterAddress::ACC_PWR_CONF, 0x00));
        // Turn on the accelerometer.
        core::utility::assert_always(write_with_confirm(RegisterAddress::ACC_PWR_CTRL, 0x04));
        board_delay_ms(1); // Datasheet: wait >=450us after entering normal mode

        spi_.unlock();
    }

    void data_ready_callback() { read(RegisterAddress::ACC_X_LSB, 6); }

private:
    enum class RegisterAddress : uint8_t {
        ACC_SOFTRESET = 0x7E,
        ACC_PWR_CTRL = 0x7D,
        ACC_PWR_CONF = 0x7C,
        ACC_SELF_TEST = 0x6D,
        INT_MAP_DATA = 0x58,
        INT2_IO_CTRL = 0x54,
        INT1_IO_CTRL = 0x53,
        ACC_RANGE = 0x41,
        ACC_CONF = 0x40,
        TEMP_LSB = 0x23,
        TEMP_MSB = 0x22,
        ACC_INT_STAT_1 = 0x1D,
        SENSORTIME_2 = 0x1A,
        SENSORTIME_1 = 0x19,
        SENSORTIME_0 = 0x18,
        ACC_Z_MSB = 0x17,
        ACC_Z_LSB = 0x16,
        ACC_Y_MSB = 0x15,
        ACC_Y_LSB = 0x14,
        ACC_X_MSB = 0x13,
        ACC_X_LSB = 0x12,
        ACC_STATUS = 0x03,
        ACC_ERR_REG = 0x02,
        ACC_CHIP_ID = 0x00,
    };

    struct __attribute__((packed)) Data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    bool write(RegisterAddress address, uint8_t value) {
        if (!spi_.try_lock())
            return false;

        spi_.transmit_receive(*this, prepare_tx_buffer_write(address, value));
        return true;
    }

    bool read(RegisterAddress address, size_t read_size) {
        if (!spi_.try_lock())
            return false;

        spi_.transmit_receive(*this, prepare_tx_buffer_read(address, read_size));
        return true;
    }

    void transmit_receive_completed_callback(size_t size) override {
        core::utility::assert_debug(size == sizeof(Data) + 2);
        auto& data = *std::launder(reinterpret_cast<Data*>(spi_.rx_buffer + 2));
        handle_uplink(usb::vendor->serializer(), data);
        spi_.unlock();
    }

    std::size_t prepare_tx_buffer_write(RegisterAddress address, uint8_t value) {
        spi_.tx_buffer[0] = static_cast<std::byte>(address);
        spi_.tx_buffer[1] = static_cast<std::byte>(value);
        return 2;
    }

    std::size_t prepare_tx_buffer_read(RegisterAddress address, size_t read_size) {
        spi_.tx_buffer[0] = std::byte{0x80} | static_cast<std::byte>(address);
        std::memset(&spi_.tx_buffer[1], 0, read_size + 1);
        return read_size + 2;
    }

    static void handle_uplink(core::protocol::Serializer& serializer, Data& data) {
        core::utility::assert_debug(
            serializer.write_imu_accelerometer(
                data::AccelerometerDataView{.x = data.x, .y = data.y, .z = data.z})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    Spi& spi_;
};

inline Accelerometer::Lazy accelerometer(
    &spi::spi2, ChipSelectPin{.gpio_base = HPM_GPIO0_BASE, .port = GPIO_DO_GPIOB, .pin = 14});

} // namespace librmcs::firmware::spi::bmi088

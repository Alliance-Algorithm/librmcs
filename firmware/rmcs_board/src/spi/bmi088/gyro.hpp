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

class Gyroscope final
    : private ISpiModule
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gyroscope, Spi::Lazy*, ChipSelectPin>;

    enum class DataRange : uint8_t {
        _2000 = 0x00,
        _1000 = 0x01,
        _500 = 0x02,
        _250 = 0x03,
        _125 = 0x04,
    };
    enum class DataRateAndBandwidth : uint8_t {
        _2000_532 = 0x00,
        _2000_230 = 0x01,
        _1000_116 = 0x02,
        _400_47 = 0x03,
        _200_23 = 0x04,
        _100_12 = 0x05,
        _200_64 = 0x06,
        _100_32 = 0x07,
    };

    explicit Gyroscope(
        Spi::Lazy* spi, ChipSelectPin chip_select, DataRange range = DataRange::_2000,
        DataRateAndBandwidth rate = DataRateAndBandwidth::_2000_230)
        : ISpiModule(chip_select)
        , spi_(spi->init()) {

        core::utility::assert_debug(spi_.try_lock());

        auto read_blocked = [this](RegisterAddress address) {
            spi_.transmit_receive_blocked(*this, prepare_tx_buffer_read(address, 1));
            return static_cast<uint8_t>(spi_.rx_buffer[1]);
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

        // Reset all registers to reset value.
        write_blocked(RegisterAddress::GYRO_SOFTRESET, 0xB6);
        board_delay_ms(30);

        // "Who am I" check.
        core::utility::assert_always(read_with_confirm(RegisterAddress::GYRO_CHIP_ID, 0x0F));

        // Enable the new data interrupt.
        core::utility::assert_always(write_with_confirm(RegisterAddress::GYRO_INT_CTRL, 0x80));

        // Set both INT3 and INT4 as push-pull, active-low, even though only INT3 is used.
        core::utility::assert_always(
            write_with_confirm(RegisterAddress::INT3_INT4_IO_CONF, 0b0000));
        // Map data ready interrupt to INT3 pin.
        core::utility::assert_always(write_with_confirm(RegisterAddress::INT3_INT4_IO_MAP, 0x01));

        // Set ODR (output data rate, Hz) and filter bandwidth (Hz).
        core::utility::assert_always(
            write_with_confirm(RegisterAddress::GYRO_BANDWIDTH, 0x80 | static_cast<uint8_t>(rate)));
        // Set data range.
        core::utility::assert_always(
            write_with_confirm(RegisterAddress::GYRO_RANGE, static_cast<uint8_t>(range)));

        // Switch the main power mode into normal mode.
        core::utility::assert_always(write_with_confirm(RegisterAddress::GYRO_LPM1, 0x00));

        spi_.unlock();
    }

    void data_ready_callback() { read(RegisterAddress::RATE_X_LSB, 6); }

private:
    enum class RegisterAddress : uint8_t {
        GYRO_SELF_TEST = 0x3C,
        INT3_INT4_IO_MAP = 0x18,
        INT3_INT4_IO_CONF = 0x16,
        GYRO_INT_CTRL = 0x15,
        GYRO_SOFTRESET = 0x14,
        GYRO_LPM1 = 0x11,
        GYRO_BANDWIDTH = 0x10,
        GYRO_RANGE = 0x0F,
        GYRO_INT_STAT_1 = 0x0A,
        RATE_Z_MSB = 0x07,
        RATE_Z_LSB = 0x06,
        RATE_Y_MSB = 0x05,
        RATE_Y_LSB = 0x04,
        RATE_X_MSB = 0x03,
        RATE_X_LSB = 0x02,
        GYRO_CHIP_ID = 0x00,
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
        core::utility::assert_debug(size == sizeof(Data) + 1);
        auto& data = *std::launder(reinterpret_cast<Data*>(spi_.rx_buffer + 1));
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
        std::memset(&spi_.tx_buffer[1], 0, read_size);
        return read_size + 1;
    }

    static void handle_uplink(core::protocol::Serializer& serializer, Data& data) {
        core::utility::assert_debug(
            serializer.write_imu_gyroscope(
                data::GyroscopeDataView{.x = data.x, .y = data.y, .z = data.z})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    Spi& spi_;
};

inline Gyroscope::Lazy gyroscope(
    &spi::spi2, ChipSelectPin{.gpio_base = HPM_GPIO0_BASE, .port = GPIO_DO_GPIOB, .pin = 10});

} // namespace librmcs::firmware::spi::bmi088

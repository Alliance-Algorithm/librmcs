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
    : private SpiModule
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Accelerometer, Spi::Lazy*, ChipSelectPin>;

    enum class Range : uint8_t { k3G = 0x00, k6G = 0x01, k12G = 0x02, k24G = 0x03 };
    enum class DataRate : uint8_t {
        k12Hz = 0x05,
        k25Hz = 0x06,
        k50Hz = 0x07,
        k100Hz = 0x08,
        k200Hz = 0x09,
        k400Hz = 0x0A,
        k800Hz = 0x0B,
        k1600Hz = 0x0C,
    };

    explicit Accelerometer(
        Spi::Lazy* spi, ChipSelectPin chip_select, Range range = Range::k6G,
        DataRate data_rate = DataRate::k1600Hz)
        : SpiModule(chip_select)
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
        read_blocked(RegisterAddress::kAccChipId);
        board_delay_ms(1);

        // Reset all registers to reset value.
        write_blocked(RegisterAddress::kAccSoftreset, 0xB6);
        board_delay_ms(1);

        // "Who am I" check.
        core::utility::assert_always(read_with_confirm(RegisterAddress::kAccChipId, 0x1E));

        // Enable INT1 as output pin, push-pull, active-low.
        core::utility::assert_always(write_with_confirm(RegisterAddress::kInt1IoCtrl, 0b00001000));
        // Map data ready interrupt to pin INT1.
        core::utility::assert_always(write_with_confirm(RegisterAddress::kIntMapData, 0b00000100));

        // Set ODR (output data rate) = data_rate and OSR (over-sampling-ratio) = 1.
        core::utility::assert_always(write_with_confirm(
            RegisterAddress::kAccConf,
            0x80 | (0x02 << 4) | (static_cast<uint8_t>(data_rate) << 0)));
        // Set accelerometer range.
        core::utility::assert_always(
            write_with_confirm(RegisterAddress::kAccRange, static_cast<uint8_t>(range)));

        // Switch the accelerometer into active mode.
        core::utility::assert_always(write_with_confirm(RegisterAddress::kAccPwrConf, 0x00));
        // Turn on the accelerometer.
        core::utility::assert_always(write_with_confirm(RegisterAddress::kAccPwrCtrl, 0x04));
        board_delay_ms(1); // Datasheet: wait >=450us after entering normal mode

        spi_.unlock();
    }

    void data_ready_callback() { read(RegisterAddress::kAccXLsb, 6); }

private:
    enum class RegisterAddress : uint8_t {
        kAccSoftreset = 0x7E,
        kAccPwrCtrl = 0x7D,
        kAccPwrConf = 0x7C,
        kAccSelfTest = 0x6D,
        kIntMapData = 0x58,
        kInt2IoCtrl = 0x54,
        kInt1IoCtrl = 0x53,
        kAccRange = 0x41,
        kAccConf = 0x40,
        kTempLsb = 0x23,
        kTempMsb = 0x22,
        kAccIntStat1 = 0x1D,
        kSensortime2 = 0x1A,
        kSensortime1 = 0x19,
        kSensortime0 = 0x18,
        kAccZMsb = 0x17,
        kAccZLsb = 0x16,
        kAccYMsb = 0x15,
        kAccYLsb = 0x14,
        kAccXMsb = 0x13,
        kAccXLsb = 0x12,
        kAccStatus = 0x03,
        kAccErrReg = 0x02,
        kAccChipId = 0x00,
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

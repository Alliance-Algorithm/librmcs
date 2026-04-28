#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include <board.h>

#include "board_app.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/base.hpp"
#include "firmware/rmcs_board/app/src/spi/spi.hpp"
#include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

struct AccelerometerTraits {
    static constexpr std::size_t kDummyBytes = 2;

    enum class RegisterAddress : uint8_t {
        kAccSoftReset = 0x7E,
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
        kSensorTime2 = 0x1A,
        kSensorTime1 = 0x19,
        kSensorTime0 = 0x18,
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
};

class Accelerometer final
    : public AccelerometerTraits
    , private Bmi088Base<AccelerometerTraits> {
public:
    using Lazy = utility::Lazy<Accelerometer, Spi::Lazy*, GpioPin>;

    static constexpr uint16_t kFirstFrameTimestampDiffQuarterUs = 1'000'000 / 1600 * 4;

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
        Spi::Lazy* spi, const GpioPin& chip_select, Range range = Range::k6G,
        DataRate data_rate = DataRate::k1600Hz)
        : Bmi088Base(spi, chip_select) {

        core::utility::assert_debug(spi_.try_lock());

        // Dummy read to switch accelerometer to SPI mode.
        read_register(RegisterAddress::kAccChipId);
        board_delay_ms(1);

        // Reset all registers to reset value.
        write_register(RegisterAddress::kAccSoftReset, 0xB6);
        board_delay_ms(1);

        // "Who am I" check.
        core::utility::assert_always(read_and_confirm(RegisterAddress::kAccChipId, 0x1E));

        // Enable INT1 as output pin, push-pull, active-low.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kInt1IoCtrl, 0b00001000));
        // Map data ready interrupt to pin INT1.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kIntMapData, 0b00000100));

        // Set ODR (output data rate) = data_rate and OSR (over-sampling-ratio) = 1.
        core::utility::assert_always(write_and_confirm(
            RegisterAddress::kAccConf, 0x80 | (0x02 << 4) | static_cast<uint8_t>(data_rate)));
        // Set accelerometer range.
        core::utility::assert_always(
            write_and_confirm(RegisterAddress::kAccRange, static_cast<uint8_t>(range)));

        // Switch the accelerometer into active mode.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kAccPwrConf, 0x00));
        // Turn on the accelerometer.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kAccPwrCtrl, 0x04));
        board_delay_ms(1); // Datasheet: wait >=450us after entering normal mode

        spi_.unlock();
    }

    void data_ready_callback(uint32_t capture_timestamp_quarter_us) {
        if (read_async(RegisterAddress::kAccXLsb, 6)) {
            pending_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
            has_pending_capture_timestamp_ = true;
        }
    }

private:
    void transmit_receive_async_callback(std::size_t size) override {
        core::utility::assert_debug(!size || has_pending_capture_timestamp_);
        if (size && has_pending_capture_timestamp_) [[likely]] {
            auto& data = parse_rx_data(spi_.rx_buffer, size);
            handle_uplink(usb::vendor->serializer(), data, pending_capture_timestamp_quarter_us_);
        }
        has_pending_capture_timestamp_ = false;
        spi_.unlock();
    }

    void handle_uplink(
        core::protocol::Serializer& serializer, Data& data, uint32_t capture_timestamp_quarter_us) {
        const uint16_t timestamp_diff_quarter_us =
            calculate_timestamp_diff_quarter_us(capture_timestamp_quarter_us);
        const auto result = serializer.write_imu_accelerometer({
            .x = data.x,
            .y = data.y,
            .z = data.z,
            .timestamp_diff_quarter_us = timestamp_diff_quarter_us,
        });
        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess) {
            last_success_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
            has_last_success_capture_timestamp_ = true;
        }
    }

    uint16_t calculate_timestamp_diff_quarter_us(uint32_t capture_timestamp_quarter_us) const {
        if (!has_last_success_capture_timestamp_)
            return kFirstFrameTimestampDiffQuarterUs;

        return saturate_timestamp_diff_quarter_us(
            capture_timestamp_quarter_us - last_success_capture_timestamp_quarter_us_);
    }

    static uint16_t saturate_timestamp_diff_quarter_us(uint32_t timestamp_diff_quarter_us) {
        if (timestamp_diff_quarter_us > std::numeric_limits<uint16_t>::max())
            return std::numeric_limits<uint16_t>::max();
        return static_cast<uint16_t>(timestamp_diff_quarter_us);
    }

    uint32_t pending_capture_timestamp_quarter_us_ = 0;
    uint32_t last_success_capture_timestamp_quarter_us_ = 0;
    bool has_pending_capture_timestamp_ = false;
    bool has_last_success_capture_timestamp_ = false;
};

inline Accelerometer::Lazy accelerometer(&spi::spi_bmi088, board::kBmi088AccelChipSelectPin);

} // namespace librmcs::firmware::spi::bmi088

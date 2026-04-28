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

struct GyroscopeTraits {
    static constexpr std::size_t kDummyBytes = 1;

    enum class RegisterAddress : uint8_t {
        kGyroSelfTest = 0x3C,
        kInt3Int4IoMap = 0x18,
        kInt3Int4IoConf = 0x16,
        kGyroIntCtrl = 0x15,
        kGyroSoftReset = 0x14,
        kGyroLpm1 = 0x11,
        kGyroBandwidth = 0x10,
        kGyroRange = 0x0F,
        kGyroIntStat1 = 0x0A,
        kRateZMsb = 0x07,
        kRateZLsb = 0x06,
        kRateYMsb = 0x05,
        kRateYLsb = 0x04,
        kRateXMsb = 0x03,
        kRateXLsb = 0x02,
        kGyroChipId = 0x00,
    };
};

class Gyroscope final
    : public GyroscopeTraits
    , private Bmi088Base<GyroscopeTraits> {
public:
    using Lazy = utility::Lazy<Gyroscope, Spi::Lazy*, GpioPin>;

    static constexpr uint16_t kFirstFrameTimestampDiffQuarterUs = 1'000'000 / 2000 * 4;

    enum class DataRange : uint8_t {
        k2000 = 0x00,
        k1000 = 0x01,
        k500 = 0x02,
        k250 = 0x03,
        k125 = 0x04,
    };
    enum class DataRateAndBandwidth : uint8_t {
        k2000And532 = 0x00,
        k2000And230 = 0x01,
        k1000And116 = 0x02,
        k400And47 = 0x03,
        k200And23 = 0x04,
        k100And12 = 0x05,
        k200And64 = 0x06,
        k100And32 = 0x07,
    };

    explicit Gyroscope(
        Spi::Lazy* spi, const GpioPin& chip_select, DataRange range = DataRange::k2000,
        DataRateAndBandwidth rate = DataRateAndBandwidth::k2000And230)
        : Bmi088Base(spi, chip_select) {

        core::utility::assert_debug(spi_.try_lock());

        // Reset all registers to reset value.
        write_register(RegisterAddress::kGyroSoftReset, 0xB6);
        board_delay_ms(30);

        // "Who am I" check.
        core::utility::assert_always(read_and_confirm(RegisterAddress::kGyroChipId, 0x0F));

        // Enable the new data interrupt.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kGyroIntCtrl, 0x80));

        // Set both INT3 and INT4 as push-pull, active-low, even though only INT3 is used.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kInt3Int4IoConf, 0b0000));
        // Map data ready interrupt to INT3 pin.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kInt3Int4IoMap, 0x01));

        // Set ODR (output data rate, Hz) and filter bandwidth (Hz).
        core::utility::assert_always(
            write_and_confirm(RegisterAddress::kGyroBandwidth, 0x80 | static_cast<uint8_t>(rate)));
        // Set data range.
        core::utility::assert_always(
            write_and_confirm(RegisterAddress::kGyroRange, static_cast<uint8_t>(range)));

        // Switch the main power mode into normal mode.
        core::utility::assert_always(write_and_confirm(RegisterAddress::kGyroLpm1, 0x00));

        spi_.unlock();
    }

    void data_ready_callback(uint32_t capture_timestamp_quarter_us) {
        if (read_async(RegisterAddress::kRateXLsb, 6)) {
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
        const auto result = serializer.write_imu_gyroscope({
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

inline Gyroscope::Lazy gyroscope(&spi::spi_bmi088, board::kBmi088GyroChipSelectPin);

} // namespace librmcs::firmware::spi::bmi088

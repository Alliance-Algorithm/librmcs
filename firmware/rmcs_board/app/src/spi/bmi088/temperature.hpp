#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_gptmr_drv.h>
#include <hpm_gptmr_regs.h>
#include <hpm_interrupt.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "board_app.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/base.hpp"
#include "firmware/rmcs_board/app/src/spi/spi.hpp"
#include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

class Temperature final
    : public AccelerometerTraits
    , private Bmi088Base<AccelerometerTraits> {
public:
    using Lazy = utility::Lazy<Temperature, Spi::Lazy*, GpioPin>;

    static constexpr uint32_t kProbeFrequencyHz = 997;
    static constexpr uint32_t kHeartbeatPeriodQuarterUs = 1'300U * 1'000U * 4U;

    explicit Temperature(Spi::Lazy* spi, const GpioPin& chip_select)
        : Bmi088Base(spi, chip_select) {
        gptmr_channel_config_t config;
        gptmr_channel_get_default_config(kTimer, &config);

        clock_add_to_group(kTimerClockName, 0);
        const uint32_t gptmr_frequency = clock_get_frequency(kTimerClockName);
        core::utility::assert_always(gptmr_frequency != 0);

        config.reload = (gptmr_frequency + (kProbeFrequencyHz / 2U)) / kProbeFrequencyHz;
        core::utility::assert_always(config.reload != 0);

        gptmr_channel_config(kTimer, kTimerChannel, &config, false);
        gptmr_enable_irq(kTimer, GPTMR_CH_RLD_IRQ_MASK(kTimerChannel));
        intc_m_enable_irq_with_priority(kTimerIrq, 1);
        gptmr_start_counter(kTimer, kTimerChannel);
    }

    void timer_callback(uint32_t capture_timestamp_quarter_us) {
        const utility::InterruptLockGuard guard;
        pending_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
        has_pending_capture_timestamp_ = true;
    }

    bool service_pending_read() {
        const utility::InterruptLockGuard guard;
        if (!has_pending_capture_timestamp_)
            return false;
        if (!read_async(RegisterAddress::kTempMsb, kTemperatureReadSizeBytes))
            return false;

        active_capture_timestamp_quarter_us_.store(
            pending_capture_timestamp_quarter_us_, std::memory_order_relaxed);
        has_active_capture_timestamp_.store(true, std::memory_order_release);
        has_pending_capture_timestamp_ = false;
        return true;
    }

private:
    static inline GPTMR_Type* const kTimer = HPM_GPTMR1;
    static constexpr clock_name_t kTimerClockName = clock_gptmr1;
    static constexpr uint32_t kTimerIrq = IRQn_GPTMR1;
    static constexpr uint8_t kTimerChannel = 0;
    static constexpr std::size_t kTemperatureReadSizeBytes = 2;

    void transmit_receive_async_callback(std::size_t size) override {
        uint32_t active_capture_timestamp_quarter_us = 0;
        const bool has_active_capture_timestamp =
            has_active_capture_timestamp_.exchange(false, std::memory_order_acquire);
        if (has_active_capture_timestamp) [[likely]] {
            active_capture_timestamp_quarter_us =
                active_capture_timestamp_quarter_us_.load(std::memory_order_relaxed);
        }

        core::utility::assert_debug(!size || has_active_capture_timestamp);
        if (size && has_active_capture_timestamp) [[likely]] {
            const uint16_t raw_temperature = parse_raw_temperature(spi_.rx_buffer, size);
            handle_uplink(
                usb::vendor->serializer(), raw_temperature, active_capture_timestamp_quarter_us);
        }
        spi_.unlock();
    }

    static uint16_t parse_raw_temperature(const std::byte* rx_buffer, std::size_t size) {
        core::utility::assert_debug(
            size == kTemperatureReadSizeBytes + AccelerometerTraits::kDummyBytes);

        const auto msb = std::to_integer<uint8_t>(rx_buffer[AccelerometerTraits::kDummyBytes]);
        const auto lsb = std::to_integer<uint8_t>(rx_buffer[AccelerometerTraits::kDummyBytes + 1]);
        return static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8U) | lsb);
    }

    void handle_uplink(
        core::protocol::Serializer& serializer, uint16_t raw_temperature,
        uint32_t capture_timestamp_quarter_us) {
        if (!should_report(raw_temperature, capture_timestamp_quarter_us))
            return;

        const auto result = serializer.write_imu_temperature({
            .temperature = raw_temperature,
            .timestamp_quarter_us = capture_timestamp_quarter_us,
        });
        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess) {
            last_reported_temperature_ = raw_temperature;
            last_reported_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
            has_last_report_ = true;
        }
    }

    bool should_report(uint16_t raw_temperature, uint32_t capture_timestamp_quarter_us) const {
        if (!has_last_report_)
            return true;
        if (raw_temperature != last_reported_temperature_)
            return true;
        return capture_timestamp_quarter_us - last_reported_capture_timestamp_quarter_us_
            >= kHeartbeatPeriodQuarterUs;
    }

    uint32_t pending_capture_timestamp_quarter_us_ = 0;
    std::atomic<uint32_t> active_capture_timestamp_quarter_us_{0};
    uint32_t last_reported_capture_timestamp_quarter_us_ = 0;
    uint16_t last_reported_temperature_ = 0;
    bool has_pending_capture_timestamp_ = false;
    std::atomic<bool> has_active_capture_timestamp_{false};
    bool has_last_report_ = false;
};

inline Temperature::Lazy temperature(&spi::spi_bmi088, board::kBmi088AccelChipSelectPin);

} // namespace librmcs::firmware::spi::bmi088

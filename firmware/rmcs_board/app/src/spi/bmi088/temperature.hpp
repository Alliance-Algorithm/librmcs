#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_gptmr_drv.h>
#include <hpm_gptmr_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "board_app.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/base.hpp"
#include "firmware/rmcs_board/app/src/spi/spi.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"
#include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

class Temperature final
    : public AccelerometerTraits
    , private Bmi088Base<AccelerometerTraits> {
public:
    using Lazy = utility::Lazy<Temperature, Spi::Lazy*, GpioPin>;

    static constexpr uint32_t kProbeFrequencyHz = 977U;
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

    void timer_callback() {
        const utility::InterruptLockGuard guard;
        probe_pending_ = true;
    }

    bool service_pending_read() {
        const utility::InterruptLockGuard guard;
        if (!probe_pending_)
            return false;
        if (!read_async(RegisterAddress::kTempMsb, kTemperatureReadSizeBytes))
            return false;

        // Temperature timestamps are anchored to SPI launch time, not probing-tick arrival.
        active_probe_launch_timestamp_quarter_us_.store(
            timer::Timer::timestamp_quarter_us(), std::memory_order_relaxed);
        has_active_probe_launch_timestamp_.store(true, std::memory_order_release);
        probe_pending_ = false;
        return true;
    }

private:
    static inline GPTMR_Type* const kTimer = HPM_GPTMR1;
    static constexpr clock_name_t kTimerClockName = clock_gptmr1;
    static constexpr uint32_t kTimerIrq = IRQn_GPTMR1;
    static constexpr uint8_t kTimerChannel = 0U;
    static constexpr std::size_t kTemperatureReadSizeBytes = 2;

    void transmit_receive_async_callback(std::size_t size) override {
        uint32_t active_probe_launch_timestamp_quarter_us = 0;
        const bool has_active_probe_launch_timestamp =
            has_active_probe_launch_timestamp_.exchange(false, std::memory_order_acquire);
        if (has_active_probe_launch_timestamp) [[likely]] {
            active_probe_launch_timestamp_quarter_us =
                active_probe_launch_timestamp_quarter_us_.load(std::memory_order_relaxed);
        }

        core::utility::assert_debug(!size || has_active_probe_launch_timestamp);
        if (size && has_active_probe_launch_timestamp) [[likely]] {
            const uint16_t raw_temperature = parse_raw_temperature(spi_.rx_buffer, size);
            handle_uplink(
                usb::vendor->serializer(), raw_temperature,
                active_probe_launch_timestamp_quarter_us);
        }
        spi_.unlock();
    }

    static uint32_t midpoint_timestamp_quarter_us(uint32_t start, uint32_t end) {
        return start + ((end - start) / 2U);
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
        uint32_t probe_launch_timestamp_quarter_us) {
        const bool observed_value_changed =
            !has_last_observation_ || raw_temperature != last_observed_temperature_;
        if (observed_value_changed) {
            // When the observed temperature changes, estimate the update time as the midpoint
            // between the previous and current probe launches.
            current_value_timestamp_quarter_us_ = has_last_observation_
                                                    ? midpoint_timestamp_quarter_us(
                                                          last_probe_launch_timestamp_quarter_us_,
                                                          probe_launch_timestamp_quarter_us)
                                                    : probe_launch_timestamp_quarter_us;
        }

        const bool value_differs_from_last_report =
            !has_last_report_ || raw_temperature != last_reported_temperature_;
        if (should_report(value_differs_from_last_report, probe_launch_timestamp_quarter_us)) {
            const uint32_t report_timestamp_quarter_us = value_differs_from_last_report
                                                           ? current_value_timestamp_quarter_us_
                                                           : probe_launch_timestamp_quarter_us;
            const auto result = serializer.write_imu_temperature({
                .raw_register_value = raw_temperature,
                .timestamp_quarter_us = report_timestamp_quarter_us,
            });
            core::utility::assert_debug(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            if (result == core::protocol::Serializer::SerializeResult::kSuccess) {
                last_reported_temperature_ = raw_temperature;
                last_reported_probe_launch_timestamp_quarter_us_ =
                    probe_launch_timestamp_quarter_us;
                has_last_report_ = true;
            }
        }

        last_observed_temperature_ = raw_temperature;
        last_probe_launch_timestamp_quarter_us_ = probe_launch_timestamp_quarter_us;
        has_last_observation_ = true;
    }

    bool should_report(
        bool value_differs_from_last_report, uint32_t probe_launch_timestamp_quarter_us) const {
        if (value_differs_from_last_report)
            return true;
        return has_last_report_
            && probe_launch_timestamp_quarter_us - last_reported_probe_launch_timestamp_quarter_us_
                   >= kHeartbeatPeriodQuarterUs;
    }

    bool probe_pending_ = false;
    std::atomic<uint32_t> active_probe_launch_timestamp_quarter_us_{0};
    uint32_t current_value_timestamp_quarter_us_ = 0;
    uint32_t last_probe_launch_timestamp_quarter_us_ = 0;
    uint32_t last_reported_probe_launch_timestamp_quarter_us_ = 0;
    uint16_t last_observed_temperature_ = 0;
    uint16_t last_reported_temperature_ = 0;
    bool has_last_observation_ = false;
    std::atomic<bool> has_active_probe_launch_timestamp_{false};
    bool has_last_report_ = false;
};

inline Temperature::Lazy temperature(&spi::spi_bmi088, board::kBmi088AccelChipSelectPin);

} // namespace librmcs::firmware::spi::bmi088

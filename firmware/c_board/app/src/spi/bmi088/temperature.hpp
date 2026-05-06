#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/base.hpp"
#include "firmware/c_board/app/src/spi/spi.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/c_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

class Temperature final
    : public AccelerometerTraits
    , private Bmi088Base<AccelerometerTraits> {
public:
    using Lazy = utility::Lazy<Temperature, Spi::Lazy*>;

    static constexpr uint32_t kProbeFrequencyHz = 977U;
    static constexpr timer::Timer::Duration kProbePeriod{
        (timer::Timer::kClockFrequency + (kProbeFrequencyHz / 2U)) / kProbeFrequencyHz};
    static constexpr uint32_t kHeartbeatPeriodQuarterUs = 1'300U * 1'000U * 4U;

    explicit Temperature(Spi::Lazy* spi)
        : Bmi088Base(spi, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin)
        , next_probe_deadline_(timer::timer->timepoint() + kProbePeriod) {}

    void poll_pending_probe() {
        const auto now = timer::timer->timepoint();

        const utility::InterruptLockGuard guard;
        if (probe_pending_ || !timer::timer->check_reached(next_probe_deadline_))
            return;

        probe_pending_ = true;
        next_probe_deadline_ = now + kProbePeriod;
    }

    bool service_pending_read() {
        const utility::InterruptLockGuard guard;
        if (!probe_pending_)
            return false;
        if (!read_async(RegisterAddress::kTempMsb, kTemperatureReadSizeBytes))
            return false;

        // Temperature timestamps are anchored to SPI launch time, not probe-deadline arrival.
        active_probe_launch_timestamp_quarter_us_.store(
            timer::timer->timepoint().time_since_epoch().count(), std::memory_order_relaxed);
        has_active_probe_launch_timestamp_.store(true, std::memory_order_release);
        probe_pending_ = false;
        return true;
    }

private:
    static constexpr std::size_t kTemperatureReadSizeBytes = 2;

    void transmit_receive_async_callback(size_t size) override {
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

    static uint16_t parse_raw_temperature(const uint8_t* rx_buffer, std::size_t size) {
        core::utility::assert_debug(
            size == kTemperatureReadSizeBytes + AccelerometerTraits::kDummyBytes);

        const auto msb = rx_buffer[AccelerometerTraits::kDummyBytes];
        const auto lsb = rx_buffer[AccelerometerTraits::kDummyBytes + 1];
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

    timer::Timer::TimePoint next_probe_deadline_;
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

inline constinit Temperature::Lazy temperature(&spi::spi1);

} // namespace librmcs::firmware::spi::bmi088

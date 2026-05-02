#pragma once

#include <chrono> // IWYU pragma: keep (https://github.com/llvm/llvm-project/issues/68213)
#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/spi/bmi088/base.hpp"
#include "firmware/c_board/app/src/spi/spi.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/c_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::bmi088 {

struct AccelerometerTraits {
    static constexpr size_t kDummyBytes = 2;

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
    using Lazy = utility::Lazy<Accelerometer, Spi::Lazy*>;

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
        Spi::Lazy* spi, Range range = Range::k6G, DataRate data_rate = DataRate::k1600Hz)
        : Bmi088Base(spi, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin) {

        using namespace std::chrono_literals;

        core::utility::assert_debug(spi_.try_lock());

        // Dummy read to switch accelerometer to SPI mode.
        read_register(RegisterAddress::kAccChipId);
        timer::timer->spin_wait(1ms);

        // Reset all registers to reset value.
        write_register(RegisterAddress::kAccSoftReset, 0xB6);
        timer::timer->spin_wait(1ms);

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
        timer::timer->spin_wait(1ms); // Datasheet: wait >=450us after entering normal mode

        spi_.unlock();
    }

    void data_ready_callback(uint32_t capture_timestamp_quarter_us) {
        const utility::InterruptLockGuard guard;
        pending_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
        has_pending_capture_timestamp_ = true;
    }

    bool service_pending_read() {
        const utility::InterruptLockGuard guard;
        if (!has_pending_capture_timestamp_)
            return false;
        if (!read_async(RegisterAddress::kAccXLsb, 6))
            return false;

        active_capture_timestamp_quarter_us_ = pending_capture_timestamp_quarter_us_;
        has_active_capture_timestamp_ = true;
        has_pending_capture_timestamp_ = false;
        return true;
    }

private:
    void transmit_receive_async_callback(size_t size) override {
        uint32_t active_capture_timestamp_quarter_us = 0;
        bool has_active_capture_timestamp = false;
        {
            const utility::InterruptLockGuard guard;
            active_capture_timestamp_quarter_us = active_capture_timestamp_quarter_us_;
            has_active_capture_timestamp = has_active_capture_timestamp_;
            has_active_capture_timestamp_ = false;
        }

        core::utility::assert_debug(!size || has_active_capture_timestamp);
        if (size && has_active_capture_timestamp) [[likely]] {
            auto& data = parse_rx_data(spi_.rx_buffer, size);
            handle_uplink(usb::vendor->serializer(), data, active_capture_timestamp_quarter_us);
        }
        spi_.unlock();
    }

    static void handle_uplink(
        core::protocol::Serializer& serializer, Data& data, uint32_t capture_timestamp_quarter_us) {
        const auto result = serializer.write_imu_accelerometer({
            .x = data.x,
            .y = data.y,
            .z = data.z,
            .timestamp_quarter_us = capture_timestamp_quarter_us,
        });
        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    uint32_t pending_capture_timestamp_quarter_us_ = 0;
    uint32_t active_capture_timestamp_quarter_us_ = 0;
    bool has_pending_capture_timestamp_ = false;
    bool has_active_capture_timestamp_ = false;
};

inline constinit Accelerometer::Lazy accelerometer(&spi1);

} // namespace librmcs::firmware::spi::bmi088

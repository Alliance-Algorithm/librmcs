#include "icm42688.hpp"

#ifdef BOARD_SPI_ICM42688

# include <algorithm>
# include <array>
# include <cstddef>
# include <cstdint>
# include <limits>
# include <optional>
# include <span>
# include <utility>

# include <hpm_clock_drv.h>
# include <hpm_common.h>
# include <hpm_gpio_drv.h>
# include <hpm_spi_drv.h>
# include <hpm_spi_regs.h>

# include <librmcs/data/datas.hpp>

# include "core/src/protocol/serializer.hpp"
# include "core/src/utility/assert.hpp"
# include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"
# include "firmware/rmcs_board/app/src/peripherals.hpp"
# include "firmware/rmcs_board/app/src/timer/timer.hpp"
# include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"

namespace librmcs::firmware::spi::icm42688 {

Icm42688::Icm42688(uintptr_t spi_base, const GpioPin& chip_select, const GpioPin& data_ready)
    : spi_(reinterpret_cast<SPI_Type*>(spi_base))
    , chip_select_(chip_select)
    , data_ready_(data_ready) {
    init_spi();
    spi_master_get_default_control_config(&spi_control_);
    spi_control_.common_config.trans_mode = spi_trans_write_read_together;

    data_ready_.configure_controller();
    data_ready_.configure_ioc_function();
    data_ready_.configure_as_input();
    data_ready_.configure_interrupt(gpio_interrupt_trigger_edge_rising);
    // Leave BMI088 and the transport usable if the second IMU is absent or fails to initialize.
    if (!init_sensor())
        return;

    data_ready_.clear_interrupt_flag();
    data_ready_.enable_interrupt();
    ready_ = true;
}

void Icm42688::poll() {
    if (!ready_)
        return;
    if (hxy_) {
        poll_hxy();
        return;
    }
    const auto timestamp = take_data_ready();
    if (!timestamp)
        return;

    // One SPI burst gets temperature, accelerometer and gyroscope from the same update.
    std::array<uint8_t, 15> tx{};
    std::array<uint8_t, tx.size()> rx{};
    tx[0] = static_cast<uint8_t>(Register::kTempDataHigh) | kReadBit;
    if (!transfer(tx, rx))
        return;

    if (newer_sample_pending())
        return;

    const data::ImuIcm42688DataView sample{
        .acceleration = {read_be16(rx, 3), read_be16(rx,  5), read_be16(rx,  7)},
        .angular_velocity = {read_be16(rx, 9), read_be16(rx, 11), read_be16(rx, 13)},
        .raw_temperature = read_be16(rx, 1),
        .timestamp_quarter_us = *timestamp,
    };
    constexpr auto invalid = std::numeric_limits<int16_t>::min();
    if (std::ranges::contains(sample.acceleration, invalid)
        || std::ranges::contains(sample.angular_velocity, invalid))
        return;

    const auto result = Peripherals::uplink_serializer().write_imu_icm42688(sample);
    core::utility::assert_debug(
        result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
}

void Icm42688::irq_handler() {
    if (ready_ && data_ready_.check_clear_interrupt_flag())
        pending_timestamp_ = timer::Timer::timestamp_quarter_us();
}

void Icm42688::init_spi() {
    chip_select_.configure_controller();
    chip_select_.set_active(false);
    chip_select_.configure_as_output();
    chip_select_.configure_ioc_function();

    spi_timing_config_t timing{};
    timing.master_config.clk_src_freq_in_hz = init_hardware(spi_);
    timing.master_config.sclk_freq_in_hz = kSpiClockHz;
    timing.master_config.cs2sclk = spi_cs2sclk_half_sclk_1;
    timing.master_config.csht = spi_csht_half_sclk_1;
    core::utility::assert_always(spi_master_timing_init(spi_, &timing) == status_success);

    spi_format_config_t format{};
    spi_master_get_default_format_config(&format);
    format.common_config.data_len_in_bits = 8;
    format.common_config.cpol = spi_sclk_low_idle;
    format.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    spi_format_init(spi_, &format);
}

bool Icm42688::init_sensor() {
    // HXY uses a different register map despite the similar part name.
    if (read_register(Register::kHxyWhoAmI) == kHxyWhoAmI) {
        hxy_ = true;
        return init_hxy();
    }
    constexpr uint8_t soft_reset = 0x01;
    // USB DFU and watchdog resets reset the MCU but can leave the IMU powered.
    if (!write_register(Register::kBankSelect, 0x00) || read_register(Register::kWhoAmI) != kWhoAmI
        || !write_register(Register::kDeviceConfig, soft_reset))
        return false;

    clock_cpu_delay_us(2'000);
    if (read_register(Register::kWhoAmI) != kWhoAmI)
        return false;

    constexpr uint8_t max_range_1khz = 0x06; // +/-16 g, +/-2000 deg/s, 1 kHz for both.
    constexpr uint8_t int1_active_high_push_pull = 0x03;
    constexpr uint8_t int1_data_ready = 0x08;
    constexpr uint8_t accel_gyro_low_noise = 0x0F;
    if (!write_and_confirm(Register::kGyroConfig, max_range_1khz)
        || !write_and_confirm(Register::kAccelConfig, max_range_1khz)
        || !write_and_confirm(Register::kInterfaceConfig0, 0x33) // Big-endian data, SPI only.
        || !write_and_confirm(Register::kIntConfig, int1_active_high_push_pull)
        || !write_and_confirm(Register::kIntConfig1, 0x00) // Clear INT_ASYNC_RESET per datasheet.
        || !write_and_confirm(Register::kIntSource0, int1_data_ready)
        || !write_register(Register::kPowerManagement, accel_gyro_low_noise))
        return false;

    clock_cpu_delay_us(45'000);
    return read_register(Register::kPowerManagement) == accel_gyro_low_noise;
}

bool Icm42688::init_hxy() {
    // HXY requires power enable first, then a 10 ms wait before other configuration.
    if (!write_register(Register::kHxyPowerControl, kHxyPowerEnable))
        return false;
    clock_cpu_delay_us(10'000);
    // BDU with auto-increment; 400 Hz accel/gyro, +/-16 g and +/-2000 deg/s.
    // INT1 selects active-high, push-pull gyro data-ready.
    if (!configure_hxy(Register::kHxyCommunicationConfig, 0x50)
        || !configure_hxy(Register::kHxyAccelConfig, 0xAA)
        || !configure_hxy(Register::kHxyAccelRange, 0x03)
        || !configure_hxy(Register::kHxyGyroConfig, 0xAA)
        || !configure_hxy(Register::kHxyGyroRange, 0x00)
        || !configure_hxy(Register::kHxyIntConfig1, 0x03))
        return false;
    clock_cpu_delay_us(50'000);
    return read_register(Register::kHxyPowerControl) == kHxyPowerEnable;
}

void Icm42688::poll_hxy() {
    // Poll the ready flags independently of INT1 to validate the sensor and its interrupt path.
    const auto now = timer::Timer::timestamp_quarter_us();
    if (now - last_hxy_poll_ < kHxyPollPeriodQuarterUs)
        return;
    last_hxy_poll_ = now;
    const auto timestamp = take_data_ready();
    // DATA_STAT, accel, gyro, sensor time, FIFO metadata and temperature in one burst.
    std::array<uint8_t, 26> tx{};
    std::array<uint8_t, tx.size()> rx{};
    tx[0] = static_cast<uint8_t>(Register::kHxyDataStatus) | kReadBit;
    if (!transfer(tx, rx))
        return;
    constexpr uint8_t error_mask = 0x30;
    constexpr uint8_t accel_gyro_ready = 0x03;
    // Independent ready flags need not assert simultaneously; a status read consumes them.
    hxy_ready_flags_ |= rx[1] & accel_gyro_ready;
    if ((rx[1] & error_mask) != 0 || hxy_ready_flags_ != accel_gyro_ready)
        return;
    hxy_ready_flags_ = 0;
    const data::ImuIcm42688DataView sample{
        .acceleration = {read_be16(rx, 2), read_be16(rx,  4), read_be16(rx,  6)},
        .angular_velocity = {read_be16(rx, 8), read_be16(rx, 10), read_be16(rx, 12)},
        .raw_temperature = read_be16(rx, 24),
        .timestamp_quarter_us = timestamp.value_or(now),
        .variant = data::Icm42688Variant::kHxy,
    };
    const auto result = Peripherals::uplink_serializer().write_imu_icm42688(sample);
    core::utility::assert_debug(
        result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
}

bool Icm42688::transfer(std::span<uint8_t> tx, std::span<uint8_t> rx) {
    chip_select_.set_active(true);
    const auto status = spi_transfer(
        spi_, &spi_control_, nullptr, nullptr, tx.data(), static_cast<uint32_t>(tx.size()),
        rx.data(), static_cast<uint32_t>(rx.size()));
    chip_select_.set_active(false);
    if (status != status_success)
        spi_reset(spi_);
    return status == status_success;
}

bool Icm42688::write_register(Register address, uint8_t value) {
    std::array<uint8_t, 2> tx{static_cast<uint8_t>(address), value};
    std::array<uint8_t, tx.size()> rx{};
    return transfer(tx, rx);
}

std::optional<uint8_t> Icm42688::read_register(Register address) {
    std::array<uint8_t, 2> tx{static_cast<uint8_t>(static_cast<uint8_t>(address) | kReadBit), 0};
    std::array<uint8_t, tx.size()> rx{};
    if (!transfer(tx, rx))
        return std::nullopt;
    return rx[1];
}

bool Icm42688::write_and_confirm(Register address, uint8_t value) {
    return write_register(address, value) && read_register(address) == value;
}

bool Icm42688::configure_hxy(Register address, uint8_t value) {
    if (!write_register(address, value))
        return false;
    clock_cpu_delay_us(1'000);
    return read_register(address) == value;
}

std::optional<uint32_t> Icm42688::take_data_ready() {
    const utility::InterruptLockGuard guard;
    return std::exchange(pending_timestamp_, std::nullopt);
}

bool Icm42688::newer_sample_pending() {
    const utility::InterruptLockGuard guard;
    // A newer update during the read may have overwritten the registers. Keep it pending.
    return pending_timestamp_.has_value()
        || gpio_check_pin_interrupt_flag(
               data_ready_.gpio_instance(), data_ready_.port(), data_ready_.pin());
}

int16_t Icm42688::read_be16(std::span<const uint8_t> bytes, std::size_t offset) {
    return static_cast<int16_t>((static_cast<uint32_t>(bytes[offset]) << 8U) | bytes[offset + 1]);
}

} // namespace librmcs::firmware::spi::icm42688

#endif

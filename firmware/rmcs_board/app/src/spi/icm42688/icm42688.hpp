#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <hpm_common.h> // IWYU pragma: keep (defines register access qualifiers)
#include <hpm_spi_drv.h>

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::spi::icm42688 {

class Icm42688 final {
public:
    using Lazy = utility::Lazy<Icm42688, uintptr_t, GpioPin, GpioPin>;

    Icm42688(uintptr_t spi_base, const GpioPin& chip_select, const GpioPin& data_ready);

    void poll();
    void irq_handler();

private:
    enum class Register : uint8_t {
        kHxyWhoAmI = 0x01,
        kHxyCommunicationConfig = 0x05,
        kHxyIntConfig1 = 0x06,
        kHxyDataStatus = 0x0B,
        kDeviceConfig = 0x11,
        kIntConfig = 0x14,
        kTempDataHigh = 0x1D,
        kHxyAccelConfig = 0x40,
        kHxyAccelRange = 0x41,
        kHxyGyroConfig = 0x42,
        kHxyGyroRange = 0x43,
        kInterfaceConfig0 = 0x4C,
        kPowerManagement = 0x4E,
        kGyroConfig = 0x4F,
        kAccelConfig = 0x50,
        kIntConfig1 = 0x64,
        kIntSource0 = 0x65,
        kWhoAmI = 0x75,
        kBankSelect = 0x76,
        kHxyPowerControl = 0x7D,
    };

    void init_spi();
    static uint32_t init_hardware(SPI_Type* spi);
    bool init_sensor();
    bool init_hxy();
    void poll_hxy();
    bool transfer(std::span<uint8_t> tx, std::span<uint8_t> rx);
    bool write_register(Register address, uint8_t value);
    std::optional<uint8_t> read_register(Register address);
    bool write_and_confirm(Register address, uint8_t value);
    bool configure_hxy(Register address, uint8_t value);
    std::optional<uint32_t> take_data_ready();
    bool newer_sample_pending();

    static int16_t read_be16(std::span<const uint8_t> bytes, std::size_t offset);

    static constexpr uint32_t kSpiClockHz = 6'000'000;
    static constexpr uint8_t kReadBit = 0x80;
    static constexpr uint8_t kWhoAmI = 0x47;
    static constexpr uint8_t kHxyWhoAmI = 0x6A;
    static constexpr uint8_t kHxyPowerEnable = 0x0E;
    static constexpr uint32_t kHxyPollPeriodQuarterUs = 1'000;

    SPI_Type* const spi_;
    const GpioPin chip_select_;
    const GpioPin data_ready_;
    spi_control_config_t spi_control_{};
    std::optional<uint32_t> pending_timestamp_;
    bool ready_ = false;
    bool hxy_ = false;
    uint32_t last_hxy_poll_ = 0;
    uint8_t hxy_ready_flags_ = 0;
};

#ifdef BOARD_SPI_ICM42688
inline constinit Icm42688::Lazy sensor{
    BOARD_SPI_ICM42688(HPM_SPI, _BASE), board::kIcm42688ChipSelectPin, board::kIcm42688IntPin};
#endif

} // namespace librmcs::firmware::spi::icm42688

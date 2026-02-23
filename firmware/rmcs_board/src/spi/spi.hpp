#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <board.h>
#include <hpm_common.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_drv.h>
#include <hpm_spi_regs.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/src/utility/lazy.hpp"

namespace librmcs::firmware::spi {

struct ChipSelectPin {
    uintptr_t gpio_base;
    uint32_t port;
    uint8_t pin;
    bool active_low = true;
};

class SpiModule {
public:
    friend class Spi;
    explicit SpiModule(ChipSelectPin chip_select_pin)
        : chip_select_pin_(chip_select_pin) {}

    SpiModule(const SpiModule&) = delete;
    SpiModule& operator=(const SpiModule&) = delete;
    SpiModule(SpiModule&&) = delete;
    SpiModule& operator=(SpiModule&&) = delete;
    virtual ~SpiModule() = default;

protected:
    virtual void transmit_receive_async_callback(std::byte* rx_buffer, std::size_t size) = 0;

    ChipSelectPin chip_select_pin_;
};

class Spi : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Spi, uintptr_t, uint32_t, uint32_t>;

    explicit Spi(uintptr_t spi_base, uint32_t irq_num, uint32_t serial_clock_frequency)
        : spi_base_(reinterpret_cast<SPI_Type*>(spi_base))
        , irq_num_(irq_num) {

        board_init_spi_pins(spi_base_);

        spi_timing_config_t timing{};
        timing.master_config.clk_src_freq_in_hz = board_init_spi_clock(spi_base_);
        timing.master_config.sclk_freq_in_hz = serial_clock_frequency;
        timing.master_config.cs2sclk = spi_cs2sclk_half_sclk_1;
        timing.master_config.csht = spi_csht_half_sclk_1;
        core::utility::assert_always(spi_master_timing_init(spi_base_, &timing) == status_success);

        spi_format_config_t format{};
        spi_master_get_default_format_config(&format);
        format.common_config.data_len_in_bits = 8;
        format.common_config.data_merge = false;
        format.common_config.lsb = false;
        format.common_config.mode = spi_master_mode;
        format.common_config.cpol = spi_sclk_high_idle;
        format.common_config.cpha = spi_sclk_sampling_even_clk_edges;
        spi_format_init(spi_base_, &format);

        spi_master_get_default_control_config(&control_config_);
        control_config_.common_config.trans_mode = spi_trans_write_read_together;
        control_config_.common_config.data_phase_fmt = spi_single_io_mode;
        control_config_.master_config.cmd_enable = false;
        control_config_.master_config.addr_enable = false;
        control_config_.master_config.token_enable = false;

        intc_m_enable_irq_with_priority(irq_num_, 1);
        spi_enable_interrupt(spi_base_, spi_end_int);

        core::utility::assert_debug_lazy([&]() noexcept {
            return spi_get_tx_fifo_size(spi_base_) == kHardwareFifoSize
                && spi_get_rx_fifo_size(spi_base_) == kHardwareFifoSize;
        });
    }

    ~Spi() = delete;
    Spi(const Spi&) = delete;
    Spi& operator=(const Spi&) = delete;
    Spi(Spi&&) = delete;
    Spi& operator=(Spi&&) = delete;

    bool is_locked() const { return locking_.test(std::memory_order::relaxed); }

    bool try_lock() { return !locking_.test_and_set(std::memory_order::relaxed); }

    void transmit_receive(SpiModule& module, std::size_t size) {
        intc_m_disable_irq(irq_num_);

        transmit_receive_async(module, size);
        while (spi_is_active(spi_base_))
            ;

        finish_transfer();

        intc_m_enable_irq(irq_num_);
    }

    void transmit_receive_async(SpiModule& module, std::size_t size) {
        core::utility::assert_debug(size <= kMaxTransferSize);
        core::utility::assert_debug_lazy(
            [&]() noexcept { return is_locked() && !spi_is_active(spi_base_); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            spi_control_init(spi_base_, &control_config_, size, size) == status_success);
        core::utility::assert_debug(
            spi_write_command(spi_base_, spi_master_mode, &control_config_, nullptr)
            == status_success);

        if (size <= kHardwareFifoSize) {
            for (size_t i = 0; i < size; i++)
                spi_base_->DATA = static_cast<uint32_t>(tx_buffer[i]);
        } else {
            core::utility::assert_failed_always(); // TODO: Transfer with DMA
        }
    }

    void transmit_receive_async_callback() {
        if (auto* module = finish_transfer())
            module->transmit_receive_async_callback(rx_buffer, tx_rx_size_);
    }

    void unlock() {
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked(); });
        locking_.clear(std::memory_order::relaxed);
    }

    static constexpr std::size_t kMaxTransferSize = HPM_L1C_CACHELINE_SIZE;
    static constexpr std::size_t kHardwareFifoSize = 8;

    alignas(HPM_L1C_CACHELINE_SIZE) std::byte tx_buffer[HPM_L1C_CACHELINE_SIZE];
    alignas(HPM_L1C_CACHELINE_SIZE) std::byte rx_buffer[HPM_L1C_CACHELINE_SIZE];

private:
    void begin_transfer(SpiModule& module, std::size_t size) {
        module_ = &module;
        tx_rx_size_ = size;

        const auto& cs = module_->chip_select_pin_;
        auto* gpio = reinterpret_cast<GPIO_Type*>(cs.gpio_base);
        gpio_write_pin(gpio, cs.port, cs.pin, cs.active_low ? 0 : 1);
    }

    SpiModule* finish_transfer() {
        if (!module_)
            return nullptr;

        const auto& cs = module_->chip_select_pin_;
        auto* gpio = reinterpret_cast<GPIO_Type*>(cs.gpio_base);
        gpio_write_pin(gpio, cs.port, cs.pin, cs.active_low ? 1 : 0);

        for (size_t i = 0; i < tx_rx_size_; i++)
            rx_buffer[i] = static_cast<std::byte>(spi_base_->DATA);

        auto* module = module_;
        module_ = nullptr;
        return module;
    }

    SPI_Type* spi_base_;
    uint32_t irq_num_;

    std::atomic_flag locking_;

    SpiModule* module_ = nullptr;
    size_t tx_rx_size_ = 0;

    spi_control_config_t control_config_{};
};

inline constinit Spi::Lazy spi2{HPM_SPI2_BASE, IRQn_SPI2, 10'000'000}; // 10Mbps

} // namespace librmcs::firmware::spi

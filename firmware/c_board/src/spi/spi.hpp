#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <gpio.h>
#include <main.h>
#include <spi.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::spi {

class SpiModule : private core::utility::Immovable {
public:
    friend class Spi;
    SpiModule(GPIO_TypeDef* chip_select_port, uint16_t chip_select_pin)
        : chip_select_port_(chip_select_port)
        , chip_select_pin_(chip_select_pin) {}

    SpiModule(const SpiModule&) = delete;
    SpiModule& operator=(const SpiModule&) = delete;
    SpiModule(SpiModule&&) = delete;
    SpiModule& operator=(SpiModule&&) = delete;

    virtual ~SpiModule() = default;

protected:
    virtual void transmit_receive_async_callback(uint8_t* rx_buffer, size_t size) = 0;

    GPIO_TypeDef* const chip_select_port_;
    const uint16_t chip_select_pin_;
};

class Spi : private core::utility::Immovable {
public:
    static constexpr size_t kMaxTransferSize = 32;

    using Lazy = utility::Lazy<Spi, SPI_HandleTypeDef*>;

    explicit Spi(SPI_HandleTypeDef* hal_spi_handle)
        : hal_spi_handle_(hal_spi_handle) {}

    bool is_locked() const { return locking_.test(std::memory_order::relaxed); }

    bool try_lock() { return !locking_.test_and_set(std::memory_order::relaxed); }

    void unlock() {
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked(); });
        locking_.clear(std::memory_order::relaxed);
    }

    void transmit_receive(SpiModule& module, size_t size) {
        core::utility::assert_debug(size <= kMaxTransferSize);
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            HAL_SPI_TransmitReceive(
                hal_spi_handle_, tx_buffer, rx_buffer, tx_rx_size_, HAL_MAX_DELAY)
            == HAL_OK);

        finish_transfer();
    }

    void transmit_receive_async(SpiModule& module, size_t size) {
        core::utility::assert_debug(size <= kMaxTransferSize);
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            HAL_SPI_TransmitReceive_IT(hal_spi_handle_, tx_buffer, rx_buffer, tx_rx_size_)
            == HAL_OK);
    }

    void transmit_receive_async_callback() {
        if (auto* module = finish_transfer())
            module->transmit_receive_async_callback(rx_buffer, tx_rx_size_);
    }

    alignas(4) uint8_t tx_buffer[kMaxTransferSize];
    alignas(4) uint8_t rx_buffer[kMaxTransferSize];

private:
    bool hal_ready() const { return hal_spi_handle_->State == HAL_SPI_STATE_READY; }

    void begin_transfer(SpiModule& module, size_t size) {
        spi_module_ = &module;
        tx_rx_size_ = static_cast<uint16_t>(size);

        select();
    }

    SpiModule* finish_transfer() {
        if (!spi_module_)
            return nullptr;

        deselect();

        auto* module = spi_module_;
        spi_module_ = nullptr;
        return module;
    }

    void select() {
        HAL_GPIO_WritePin(
            spi_module_->chip_select_port_, spi_module_->chip_select_pin_, GPIO_PIN_RESET);
    }

    void deselect() {
        HAL_GPIO_WritePin(
            spi_module_->chip_select_port_, spi_module_->chip_select_pin_, GPIO_PIN_SET);
    }

    SPI_HandleTypeDef* hal_spi_handle_;

    std::atomic_flag locking_;

    SpiModule* spi_module_{nullptr};
    uint16_t tx_rx_size_{0};
};

inline constinit Spi::Lazy spi1(&hspi1);

} // namespace librmcs::firmware::spi

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

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

    virtual ~SpiModule() = default;

protected:
    virtual void transmit_receive_async_callback(size_t size) = 0;

    GPIO_TypeDef* const chip_select_port_;
    const uint16_t chip_select_pin_;
};

class Spi : private core::utility::Immovable {
public:
    static constexpr size_t kMaxTransferSize = 32;

    using Lazy = utility::Lazy<Spi, SPI_HandleTypeDef*>;

    explicit Spi(SPI_HandleTypeDef* hal_spi_handle)
        : hal_spi_handle_(hal_spi_handle) {
        init_dma_transfer();
    }

    bool is_locked() const { return locking_.test(std::memory_order::acquire); }

    bool try_lock() { return !locking_.test_and_set(std::memory_order::acquire); }

    void unlock() {
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked(); });
        locking_.clear(std::memory_order::release);
    }

    void transmit_receive(SpiModule& module, size_t size) {
        core::utility::assert_debug(0 < size && size <= kMaxTransferSize);
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            HAL_SPI_TransmitReceive(
                hal_spi_handle_, tx_buffer, rx_buffer, tx_rx_size_, HAL_MAX_DELAY)
            == HAL_OK);

        finish_transfer();
    }

    void transmit_receive_async(SpiModule& module, size_t size) {
        core::utility::assert_debug(0 < size && size <= kMaxTransferSize);
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        trigger_dma();
    }

    void dma_transfer_complete_callback() {
        active_dma_count_.fetch_sub(1, std::memory_order::release);
    }

    void update() {
        if (active_dma_count_.load(std::memory_order::acquire) == 0
            && !__HAL_SPI_GET_FLAG(hal_spi_handle_, SPI_FLAG_BSY)) {
            transmit_receive_async_callback(true);
        }
    }

    void transmit_receive_async_callback(bool success) {
        // Fail-fast in debug builds
        core::utility::assert_debug_lazy([&]() noexcept { return success; });

        // Release fallback: cleanup
        if (!success) [[unlikely]] {
            HAL_DMA_Abort(hal_spi_handle_->hdmarx);
            HAL_DMA_Abort(hal_spi_handle_->hdmatx);
        }

        active_dma_count_.store(kDmaNotPerformed, std::memory_order::relaxed);

        CLEAR_BIT(hal_spi_handle_->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        hal_spi_handle_->TxXferCount = 0;
        hal_spi_handle_->RxXferCount = 0;
        hal_spi_handle_->State = HAL_SPI_STATE_READY;

        if (auto* module = finish_transfer())
            module->transmit_receive_async_callback(success ? tx_rx_size_ : 0);
    }

    alignas(4) uint8_t tx_buffer[kMaxTransferSize];
    alignas(4) uint8_t rx_buffer[kMaxTransferSize];

private:
    bool hal_ready() const { return hal_spi_handle_->State == HAL_SPI_STATE_READY; }

    void init_dma_transfer() {
        hal_spi_handle_->pTxBuffPtr = tx_buffer;
        hal_spi_handle_->pRxBuffPtr = rx_buffer;
        hal_spi_handle_->RxISR = nullptr;
        hal_spi_handle_->TxISR = nullptr;

        hal_spi_handle_->hdmarx->XferHalfCpltCallback = nullptr;
        hal_spi_handle_->hdmarx->XferCpltCallback = dma_transfer_complete_callback_global;
        hal_spi_handle_->hdmarx->XferErrorCallback = dma_error_callback_global;
        hal_spi_handle_->hdmarx->XferAbortCallback = nullptr;

        hal_spi_handle_->hdmatx->XferHalfCpltCallback = nullptr;
        hal_spi_handle_->hdmatx->XferCpltCallback = dma_transfer_complete_callback_global;
        hal_spi_handle_->hdmatx->XferErrorCallback = dma_error_callback_global;
        hal_spi_handle_->hdmatx->XferAbortCallback = nullptr;

        __HAL_SPI_ENABLE_IT(hal_spi_handle_, SPI_IT_ERR);

        if ((hal_spi_handle_->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
            __HAL_SPI_ENABLE(hal_spi_handle_);
        }
    }

    void trigger_dma() {
        hal_spi_handle_->State = HAL_SPI_STATE_BUSY_TX_RX;
        hal_spi_handle_->TxXferSize = tx_rx_size_;
        hal_spi_handle_->TxXferCount = tx_rx_size_;
        hal_spi_handle_->RxXferSize = tx_rx_size_;
        hal_spi_handle_->RxXferCount = tx_rx_size_;
        SET_BIT(hal_spi_handle_->Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        active_dma_count_.store(2, std::memory_order::relaxed);

        core::utility::assert_debug(
            HAL_DMA_Start_IT(
                hal_spi_handle_->hdmarx,
                reinterpret_cast<uintptr_t>(&hal_spi_handle_->Instance->DR),
                reinterpret_cast<uintptr_t>(rx_buffer), tx_rx_size_)
            == HAL_OK);

        core::utility::assert_debug(
            HAL_DMA_Start_IT(
                hal_spi_handle_->hdmatx, reinterpret_cast<uintptr_t>(tx_buffer),
                reinterpret_cast<uintptr_t>(&hal_spi_handle_->Instance->DR), tx_rx_size_)
            == HAL_OK);
    }

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

    static void dma_transfer_complete_callback_global(DMA_HandleTypeDef* hal_dma_handle);
    static void dma_error_callback_global(DMA_HandleTypeDef* hal_dma_handle);

    SPI_HandleTypeDef* hal_spi_handle_;

    std::atomic_flag locking_;

    static constexpr int8_t kDmaNotPerformed = std::numeric_limits<int8_t>::max();
    std::atomic<int8_t> active_dma_count_ = kDmaNotPerformed;

    SpiModule* spi_module_{nullptr};
    uint16_t tx_rx_size_{0};
};

inline constinit Spi::Lazy spi1(&hspi1);

} // namespace librmcs::firmware::spi
